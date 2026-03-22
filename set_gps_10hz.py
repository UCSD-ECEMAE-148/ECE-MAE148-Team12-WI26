"""
set_gps_10hz_multi.py — Configure SparkFun NEO-F10N for 10 Hz multi-GNSS mode.

Restores the GPS to multi-constellation (GPS + Galileo + BeiDou) at 10 Hz.

Note: The NEO-F10N does not support GLONASS — only GPS, Galileo, BeiDou.

Run ONCE, then run set_gps_messages.py to configure NMEA message output.
"""

import time
import serial
from pyubx2 import UBXMessage, SET_LAYER_RAM, SET_LAYER_BBR, TXN_NONE, UBXReader

PORT   = "/dev/ttyUSB0"
BAUD   = 38400
LAYERS = SET_LAYER_RAM | SET_LAYER_BBR


def send_and_ack(ser, msg, label):
    ser.write(msg.serialize())
    time.sleep(0.3)

    deadline = time.time() + 2.0
    ubr = UBXReader(ser)

    while time.time() < deadline:
        try:
            _, parsed = ubr.read()
            if parsed is None:
                continue
            identity = parsed.identity
            if identity == "ACK-ACK":
                print(f"  [OK]  {label}")
                return True
            elif identity == "ACK-NAK":
                print(f"  [NAK] {label} — command rejected by receiver")
                return False
        except Exception:
            pass

    print(f"  [??]  {label} — no ACK received (may still have worked)")
    return None


def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(0.5)

    print("\n--- Step 1: Re-enable Galileo and BeiDou ---")
    msg = UBXMessage.config_set(LAYERS, TXN_NONE, [
        ("CFG_SIGNAL_GAL_ENA", True),
        ("CFG_SIGNAL_BDS_ENA", True),
    ])
    send_and_ack(ser, msg, "Enable Galileo + BeiDou")

    print("\n--- Step 2: Set navigation rate to 10 Hz (100ms measurement interval) ---")
    msg = UBXMessage.config_set(LAYERS, TXN_NONE, [
        ("CFG_RATE_MEAS", 100),
        ("CFG_RATE_NAV",    1),
    ])
    send_and_ack(ser, msg, "Set 10 Hz navigation rate")

    print("\nWaiting 2s for GNSS subsystem restart...")
    time.sleep(2.0)

    print("\n--- Step 3: Verify by reading a few NMEA sentences ---")
    ser.reset_input_buffer()
    deadline = time.time() + 5.0
    count = 0
    while time.time() < deadline and count < 10:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("$"):
            print(f"  {line}")
            count += 1

    ser.close()
    print("\nDone. GPS configured for multi-GNSS (GPS + Galileo + BeiDou) at 10 Hz.")
    print("Settings saved to BBR — will survive power cycle.")
    print("Now run set_gps_messages.py to configure NMEA message output.")


if __name__ == "__main__":
    main()
