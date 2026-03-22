"""
gps_imu_fused_bno08x.py — DonkeyCar part for GPS/IMU fusion using BNO08x.

Replaces the GpsNmeaPositions + GpsLatestPosition pipeline in manage.py.
Outputs (x, y) in UTM meters at IMU rate (~50 Hz) instead of GPS rate (10 Hz),
eliminating the position freeze seen when the PID loop outruns GPS updates.

Kalman filter state: [x, y, vx, vy, yaw]
- IMU provides: linear acceleration (world frame) + yaw from quaternion
- GPS provides: position correction at 10 Hz

Drop this file into:
    ~/donkeycar/donkeycar/parts/gps_imu_fused_bno08x.py
"""

import os
os.environ["BLINKA_LOG_LEVEL"] = "ERROR"

import gc
import logging
import math
import threading
import time

import numpy as np
import pynmea2
import serial
import utm

import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)

gc.disable()

logger = logging.getLogger(__name__)


# ==============================
# TUNABLE CONSTANTS
# ==============================

# Kalman filter noise parameters (tuned for NEO-F10N + BNO08x)
SIGMA_GPS_M        = 1.5               # GPS position noise, metres
SIGMA_YAW_RAD      = math.radians(2.0) # IMU yaw noise
Q_ACCEL            = 0.4               # process noise from accelerometer
Q_YAW_RATE        = math.radians(15.0) ** 2

# Motion thresholds
COURSE_BLEND_SPEED = 0.8   # m/s — below this, GPS course is ignored
STOP_SPEED_THRESH  = 0.20  # m/s — GPS speed below which we check accel
STOP_ACCEL_THRESH  = 0.08  # m/s² — averaged accel below which vel is zeroed
ACCEL_CLAMP        = 8.0   # m/s² — reject wild IMU spikes

# IMU
IMU_HZ             = 50.0
IMU_DT_TARGET      = 1.0 / IMU_HZ
IMU_SPIKE_THRESH   = math.radians(120.0)  # garbage filter only
IMU_STALL_SEC      = 5.0   # re-enable features after this many seconds of silence

# GPS
GPS_STABILIZE_SEC  = 3.0   # seconds to wait after first fix before setting origin
R_EARTH            = 6378137.0

# Print diagnostics at this rate
PRINT_HZ           = 2.0
PRINT_DT           = 1.0 / PRINT_HZ

# Minimum speed to update course output
MIN_SPEED_MPS      = 0.05


# ==============================
# MATH HELPERS
# ==============================

def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _quat_to_yaw(qi: float, qj: float, qk: float, qr: float) -> float:
    t0 = 2.0 * (qr * qk + qi * qj)
    t1 = 1.0 - 2.0 * (qj * qj + qk * qk)
    return math.atan2(t0, t1)


def _rot_body_to_world(ax_b: float, ay_b: float, yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * ax_b - s * ay_b, s * ax_b + c * ay_b


def _nmea_course_to_enu(course_rad: float) -> float:
    return _wrap(math.pi / 2.0 - course_rad)


# ==============================
# KALMAN FILTER
# ==============================

class _KF:
    """
    5-state Kalman filter: [x, y, vx, vy, yaw]
    IMU acceleration is the control input.
    GPS position and IMU yaw are measurements.
    """

    def __init__(self):
        self.x = np.zeros((5, 1))
        self.P = np.diag([5.0, 5.0, 2.0, 2.0, math.pi ** 2])
        self.I = np.eye(5)

        self.Hgps = np.array([[1,0,0,0,0],[0,1,0,0,0]], dtype=float)
        self.Rgps = np.diag([SIGMA_GPS_M**2, SIGMA_GPS_M**2])

        self.Hyaw = np.array([[0,0,0,0,1]], dtype=float)
        self.Ryaw = np.array([[SIGMA_YAW_RAD**2]])

        # Pre-allocated to avoid GC pressure in the hot loop
        self.F = np.eye(5)
        self.B = np.zeros((5, 2))
        self.Q = np.zeros((5, 5))
        self.u = np.zeros((2, 1))
        self.z_gps = np.zeros((2, 1))
        self.z_yaw = np.zeros((1, 1))
        self.blend_Ryaw = np.zeros((1, 1))

    def predict(self, dt: float, ax: float, ay: float) -> None:
        dt2 = dt * dt
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.B[0, 0] = 0.5 * dt2
        self.B[1, 1] = 0.5 * dt2
        self.B[2, 0] = dt
        self.B[3, 1] = dt
        self.Q[0, 0] = 0.25 * dt2 * dt2 * Q_ACCEL
        self.Q[1, 1] = self.Q[0, 0]
        self.Q[2, 2] = dt2 * Q_ACCEL
        self.Q[3, 3] = self.Q[2, 2]
        self.Q[4, 4] = Q_YAW_RATE * dt2
        self.u[0, 0] = ax
        self.u[1, 0] = ay
        self.x = self.F @ self.x + self.B @ self.u
        self.x[4, 0] = _wrap(self.x[4, 0])
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.F[0, 2] = 0.0
        self.F[1, 3] = 0.0

    def update_gps(self, xg: float, yg: float) -> None:
        self.z_gps[0, 0] = xg
        self.z_gps[1, 0] = yg
        y = self.z_gps - self.Hgps @ self.x
        S = self.Hgps @ self.P @ self.Hgps.T + self.Rgps
        K = np.linalg.solve(S.T, (self.P @ self.Hgps.T).T).T
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.Hgps) @ self.P

    def update_yaw(self, yaw: float) -> None:
        self.z_yaw[0, 0] = _wrap(yaw - self.x[4, 0])
        S = self.Hyaw @ self.P @ self.Hyaw.T + self.Ryaw
        K = np.linalg.solve(S.T, (self.P @ self.Hyaw.T).T).T
        self.x = self.x + K @ self.z_yaw
        self.x[4, 0] = _wrap(self.x[4, 0])
        self.P = (self.I - K @ self.Hyaw) @ self.P


# ==============================
# DONKEYCAR PART
# ==============================

class GpsImuFusedBno08x:
    """
    DonkeyCar part: fuses BNO08x IMU with GPS NMEA to produce
    a smoothly-updated (x, y) position in UTM metres.

    Wiring in manage.py (replaces add_gps):

        from donkeycar.parts.gps_imu_fused_bno08x import GpsImuFusedBno08x

        fusion = GpsImuFusedBno08x(
            gps_port=cfg.GPS_SERIAL,
            gps_baud=cfg.GPS_SERIAL_BAUDRATE,
        )
        V.add(fusion,
              outputs=['pos/x', 'pos/y'],
              threaded=True)

    outputs:
        pos/x  — UTM easting  (metres, same coordinate system as path_follow)
        pos/y  — UTM northing (metres)
    """

    def __init__(
        self,
        gps_port: str = "/dev/ttyUSB0",
        gps_baud: int = 38400,
        debug: bool = False,
    ) -> None:
        self.gps_port = gps_port
        self.gps_baud = gps_baud
        self.debug = debug

        # Thread-safe output — what run_threaded() returns
        self._lock = threading.Lock()
        self._pos_x: float | None = None
        self._pos_y: float | None = None

        # Internal GPS state (written by GPS thread, read by fusion loop)
        self._gps_lock = threading.Lock()
        self._gps = {
            "has_fix": False,
            "lat": None,
            "lon": None,
            "speed": 0.0,
            "course": None,
            "seq": 0,
            "t": None,
        }

        self.running = True
        self._print_dt = PRINT_DT
        logger.info("GpsImuFusedBno08x created")

    # ------------------------------------------------------------------
    # DonkeyCar part interface
    # ------------------------------------------------------------------

    def update(self) -> None:
        """Runs in a background thread started by the DonkeyCar framework."""
        # Start GPS reader as a sub-thread
        threading.Thread(target=self._gps_thread, daemon=True).start()

        # Run the main fusion loop (blocks until self.running = False)
        self._fusion_loop()

    def run_threaded(self):
        """Called by the DonkeyCar vehicle loop at DRIVE_LOOP_HZ.
        Returns (pos/x, pos/y) or (None, None) before first GPS fix."""
        with self._lock:
            return self._pos_x, self._pos_y

    def shutdown(self) -> None:
        self.running = False

    # ------------------------------------------------------------------
    # GPS reader thread
    # ------------------------------------------------------------------

    def _gps_thread(self) -> None:
        while self.running:
            try:
                ser = serial.Serial(self.gps_port, self.gps_baud, timeout=0.1)
                logger.info(f"GPS connected on {self.gps_port}")
                break
            except Exception as e:
                logger.warning(f"Waiting for GPS: {e}")
                time.sleep(1.0)

        while self.running:
            try:
                line = ser.readline().decode("utf-8", errors="ignore")
                if not (line.startswith("$GNRMC") or line.startswith("$GPRMC")):
                    continue
                try:
                    msg = pynmea2.parse(line)
                except Exception:
                    continue
                if msg.status != "A":
                    continue

                speed_kn = float(msg.spd_over_grnd) if msg.spd_over_grnd else 0.0
                course = (
                    math.radians(float(msg.true_course))
                    if msg.true_course else None
                )

                with self._gps_lock:
                    self._gps["has_fix"] = True
                    self._gps["lat"]     = msg.latitude
                    self._gps["lon"]     = msg.longitude
                    self._gps["speed"]   = speed_kn * 0.514444
                    self._gps["course"]  = course
                    self._gps["seq"]    += 1
                    self._gps["t"]       = time.monotonic()

            except Exception as e:
                logger.warning(f"GPS read error: {e}")
                time.sleep(0.05)

    # ------------------------------------------------------------------
    # Main fusion loop (runs inside update())
    # ------------------------------------------------------------------

    def _fusion_loop(self) -> None:
        logging.getLogger("adafruit_bno08x").setLevel(logging.CRITICAL)

        # --- Init IMU ---
        logger.info("Initialising IMU")
        bno = self._init_imu()

        kf = _KF()

        # UTM origin — set once after GPS stabilises
        utm_x0: float | None = None
        utm_y0: float | None = None
        stabilize_start: float | None = None
        last_gps_seq = -1
        last_yaw: float | None = None
        last_t = time.monotonic()
        last_imu_ok = time.monotonic()
        last_print = 0.0
        last_course_deg = 0.0

        logger.info("Fusion loop running")

        while self.running:
            now = time.monotonic()
            dt = min(max(now - last_t, 0.001), 0.1)
            last_t = now

            # --- Read IMU ---
            ax_b = ay_b = 0.0
            try:
                time.sleep(0.001)
                qi, qj, qk, qr = bno.game_quaternion
                time.sleep(0.001)
                ax_b, ay_b, _ = bno.linear_acceleration
                last_imu_ok = time.monotonic()
            except Exception:
                if time.monotonic() - last_imu_ok > IMU_STALL_SEC:
                    logger.warning("IMU stalled — re-enabling features")
                    try:
                        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 20_000)
                        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION,  20_000)
                        last_imu_ok = time.monotonic()
                        last_yaw = None
                        logger.info("IMU re-enabled")
                    except Exception as e:
                        logger.error(f"IMU re-enable failed: {e}")
                        time.sleep(1.0)
                time.sleep(IMU_DT_TARGET)
                continue

            yaw_imu = _quat_to_yaw(qi, qj, qk, qr)

            # Spike rejection (garbage filter only — 120 deg/cycle threshold)
            if last_yaw is not None:
                if abs(_wrap(yaw_imu - last_yaw)) > IMU_SPIKE_THRESH:
                    yaw_imu = last_yaw
            last_yaw = yaw_imu

            ax_b = max(-ACCEL_CLAMP, min(ACCEL_CLAMP, ax_b))
            ay_b = max(-ACCEL_CLAMP, min(ACCEL_CLAMP, ay_b))
            ax_w, ay_w = _rot_body_to_world(ax_b, ay_b, yaw_imu)
            accel_mag = math.hypot(ax_b, ay_b)

            # --- Read GPS snapshot ---
            with self._gps_lock:
                fix    = self._gps["has_fix"]
                lat    = self._gps["lat"]
                lon    = self._gps["lon"]
                speed  = self._gps["speed"]
                course = self._gps["course"]
                seq    = self._gps["seq"]
                gps_t  = self._gps["t"]

            # --- Wait for GPS origin ---
            if utm_x0 is None:
                if fix:
                    if stabilize_start is None:
                        stabilize_start = now
                        logger.info("Stabilising GPS origin...")
                    elif now - stabilize_start > GPS_STABILIZE_SEC:
                        easting, northing, _, _ = utm.from_latlon(lat, lon)
                        utm_x0 = easting
                        utm_y0 = northing
                        kf.x[4, 0] = yaw_imu
                        logger.info(
                            f"UTM origin set: easting={utm_x0:.2f} northing={utm_y0:.2f}"
                        )
                time.sleep(IMU_DT_TARGET)
                continue

            # --- Kalman predict + IMU yaw update (every cycle at 50 Hz) ---
            kf.predict(dt, ax_w, ay_w)
            kf.update_yaw(yaw_imu)

            # --- GPS correction on each new fix ---
            if fix and seq != last_gps_seq:
                last_gps_seq = seq

                easting, northing, _, _ = utm.from_latlon(lat, lon)
                xg = easting  - utm_x0
                yg = northing - utm_y0
                kf.update_gps(xg, yg)

                if course is not None:
                    course_enu = _nmea_course_to_enu(course)
                    alpha = max(
                        0.0,
                        min(1.0, (speed - COURSE_BLEND_SPEED) / COURSE_BLEND_SPEED),
                    )
                    if alpha > 0.0:
                        kf.blend_Ryaw[0, 0] = (SIGMA_YAW_RAD**2) / (alpha + 1e-6)
                        kf.z_yaw[0, 0] = _wrap(course_enu - kf.x[4, 0])
                        S = kf.Hyaw @ kf.P @ kf.Hyaw.T + kf.blend_Ryaw
                        K = np.linalg.solve(S.T, (kf.P @ kf.Hyaw.T).T).T
                        kf.x = kf.x + K @ kf.z_yaw
                        kf.x[4, 0] = _wrap(kf.x[4, 0])
                        kf.P = (kf.I - K @ kf.Hyaw) @ kf.P

                        w = 0.3 * alpha
                        vx_gps = speed * math.cos(course_enu)
                        vy_gps = speed * math.sin(course_enu)
                        kf.x[2, 0] = (1.0 - w) * kf.x[2, 0] + w * vx_gps
                        kf.x[3, 0] = (1.0 - w) * kf.x[3, 0] + w * vy_gps

                if speed < STOP_SPEED_THRESH and accel_mag < STOP_ACCEL_THRESH:
                    kf.x[2, 0] = 0.0
                    kf.x[3, 0] = 0.0

            # --- Publish fused position ---
            x   = float(kf.x[0, 0])
            y   = float(kf.x[1, 0])
            vx  = float(kf.x[2, 0])
            vy  = float(kf.x[3, 0])
            spd = math.hypot(vx, vy)

            # pos/x and pos/y are in the same UTM frame as the recorded path.
            # DonkeyCar's path_follow records waypoints as UTM easting/northing,
            # so we output absolute UTM coordinates (not relative to origin).
            pos_x = x + utm_x0
            pos_y = y + utm_y0

            with self._lock:
                self._pos_x = pos_x
                self._pos_y = pos_y

            if spd > MIN_SPEED_MPS:
                course_enu = math.atan2(vy, vx)
                course_nmea = _wrap(math.pi / 2.0 - course_enu)
                last_course_deg = math.degrees(course_nmea) % 360.0

            # --- Debug print ---
            if self.debug and now - last_print > PRINT_DT:
                last_print = now
                gps_age = round(now - gps_t, 2) if gps_t is not None else -1.0
                yaw_deg = math.degrees(float(kf.x[4, 0]))
                print(
                    f"[fusion] pos=({pos_x:.2f},{pos_y:.2f}) "
                    f"vx={vx:.2f} vy={vy:.2f} "
                    f"yaw={yaw_deg:.1f}deg "
                    f"spd={spd:.2f} gps_seq={seq} "
                    f"gps_age={gps_age:.2f}s acc={accel_mag:.2f}",
                    flush=True,
                )

            sleep = IMU_DT_TARGET - (time.monotonic() - now)
            if sleep > 0:
                time.sleep(sleep)

    # ------------------------------------------------------------------
    # IMU init
    # ------------------------------------------------------------------

    @staticmethod
    def _init_imu() -> BNO08X_I2C:
        i2c = busio.I2C(board.SCL, board.SDA)
        bno = BNO08X_I2C(i2c)
        time.sleep(0.5)
        bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 20_000)
        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION,  20_000)
        time.sleep(0.5)
        logger.info("IMU ready")
        return bno
