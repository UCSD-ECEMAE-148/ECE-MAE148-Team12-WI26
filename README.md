![image](https://xiltrix.com/wp-content/uploads/2018/02/UCSD-Jacobs-School-of-Engineering-XiltriX.jpg)
### <div align="center"> ECE/MAE 148 Final Project </div>
#### <div align="center"> Team 12 - Winter 2026 </div>



## Table of Contents
<ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#donkeycar">DonkeyCar</a></li>
    <li><a href="#final-project-donkeycar-gpsimu-fusion--bno08x--neo-f10n">Final Project: DonkeyCar GPS/IMU Fusion — BNO08x + NEO-F10N</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#future-work">Future Work</a></li>  
    <li><a href="#final-project-video">Final Project Video</a></li>
    <li><a href="#autonomous-laps-videos">Autonomous Laps Videos</a></li>
    <li><a href="#cad-designs-for-donkeycar">CAD Designs for DonkeyCar</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
</ol>

<hr>

## Team Members
- Spencer Herberich - MAE
- Pelin Kaynak - MAE
- Woojin Lee - ECE
- Thomas Beauchataud - UPS

<hr>

## DonkeyCar
<div align="center">
    [Front View] </div>
    <div align="center">
    <img width="669" height="796" alt="Screenshot 2026-03-20 220315" src="https://github.com/user-attachments/assets/64b4d67a-8fc3-4788-9bc3-ea7d757362aa" />    
</div>

<div align="center">
    [Right View] </div>
    <div align="center">
    <img width="657" height="682" alt="Screenshot 2026-03-20 220330" src="https://github.com/user-attachments/assets/b3989641-f129-43fc-afd6-c737b09f96ea" />
</div>

<div align="center">
    [Left View] </div>
    <div align="center">
    <img width="639" height="706" alt="Screenshot 2026-03-20 220341" src="https://github.com/user-attachments/assets/c365a62e-c6ef-4e9f-9c81-e8bae11cdac6" />
</div>

<div align="center">
    [Behind View] </div>
    <div align="center">
    <img width="608" height="750" alt="Screenshot 2026-03-20 220353" src="https://github.com/user-attachments/assets/6aefa78a-0661-4010-86c7-f4573357e9e6" />
</div>

<div align="center">
    [Birdeye View] </div>
    <div align="center">
    <img width="705" height="872" alt="Screenshot 2026-03-20 220406" src="https://github.com/user-attachments/assets/8e9c3b3e-053a-4a84-a408-4e206b6fc14e" />
</div>

<hr>

## Final Project: DonkeyCar GPS/IMU Fusion — BNO08x + NEO-F10N
Fuses a **BNO08x IMU** (50 Hz) with a **SparkFun NEO-F10N GPS** (10 Hz) via a Kalman filter, giving the DonkeyCar path follower smooth 50 Hz position updates instead of freezing between GPS fixes. 

## Files

| File | What it is |
|---|---|
| `gps_imu_fused_bno08x.py` | Drop-in DonkeyCar part: copy into `donkeycar/parts/` |
| `set_gps_10hz.py` | One-time GPS receiver configuration script |

---

## Setup

> Assumes DonkeyCar is already installed with a virtual environment at `~/donkey_env`.

### 1. Install dependencies

```bash
source ~/donkey_env/bin/activate
pip install adafruit-circuitpython-bno08x pyubx2 pyserial utm
```

### 2. Copy the fusion part into DonkeyCar

```bash
cp gps_imu_fused_bno08x.py \
   ~/donkey_env/lib/python3.11/site-packages/donkeycar/parts/
```

> Adjust `python3.11` if your environment uses a different Python version.

### 3. Configure the GPS receiver (run once)

This writes 10 Hz multi-GNSS settings (GPS + Galileo + BeiDou) to the receiver's battery-backed RAM. You only need to run this once, settings survive power cycles.

```bash
python set_gps_10hz.py
```

Each step should print `[OK]`. A `[??]` means no ACK was received but the setting likely applied, the NMEA sentences printed at the end will confirm it's working.

### 4. Patch `manage.py`

Open `~/projects/mycar/manage.py` and replace the entire `add_gps` function with:

```python
def add_gps(V, cfg):
    if cfg.HAVE_GPS:
        from donkeycar.parts.gps_imu_fused_bno08x import GpsImuFusedBno08x
        fusion = GpsImuFusedBno08x(
            gps_port=cfg.GPS_SERIAL,
            gps_baud=cfg.GPS_SERIAL_BAUDRATE,
            debug=cfg.GPS_DEBUG,
        )
        V.add(fusion, outputs=['pos/x', 'pos/y'], threaded=True)
        return None
    return None
```

### 5. Update `myconfig.py`

Open `~/projects/mycar/myconfig.py` and add/update:

```python
GPS_DEBUG = True           # set False once confirmed working
GPS_SERIAL = "/dev/ttyUSB0"
GPS_SERIAL_BAUDRATE = 38400
```

### 6. Test

```bash
cd ~/projects/mycar
python manage.py drive
```

With `GPS_DEBUG = True` you should see lines like:

```
[fusion] pos=(476823.12,3652910.44) vx=0.12 vy=0.03 yaw=14.2deg spd=0.13 gps_seq=42 gps_age=0.08s acc=0.11
```

Once you see `UTM origin set` in the logs, the filter has locked on and position is being fused. Set `GPS_DEBUG = False` for normal driving.

---

## Hardware

- Raspberry Pi 5
- [SparkFun NEO-F10N GPS](https://www.sparkfun.com/products/21234) via USB → `/dev/ttyUSB0`
- [Adafruit BNO08x IMU](https://www.adafruit.com/product/4754) via I2C (SDA/SCL header pins)

---

## How It Works

The Kalman filter maintains a 5-state estimate: `[x, y, vx, vy, yaw]` in a local ENU frame anchored to a UTM origin set at first GPS fix.

- **50 Hz predict step** — BNO08x linear acceleration (rotated to world frame) propagates position and velocity forward
- **50 Hz yaw update** — BNO08x game rotation vector corrects heading
- **10 Hz GPS update** — NEO-F10N position fix corrects x/y; GPS course is blended into yaw when speed > 0.8 m/s

Output is absolute UTM coordinates, matching the frame DonkeyCar uses when recording waypoints with the path follower.

---

## License

MIT

<hr>

## What We Promised
### Must Have
* IMU implemented
* Kalman filter implemented
* Bridge between IMU, GPS, and Donkeycar

<hr>

## Accomplishments
* Smooth communication between IMU, GPS, and Donkeycar
* Completing the GPS lap with implemented IMU and cheaper GPS
* Increasing the VESC throttle to 0.4 to make the car drive faster but still maneuver smoothly

<hr>

## Challenges
* Struggled a good amount of time to figure out the delay between IMU and Donkeycar environment 

<hr>

## Future Work
* Completing the GPS laps with 100% VESC throttle while maintaining smooth maneuver

<hr>

## Final Project Video 
<div align="center">
     
#### **[Complete GPS lap with IMU]**
[<img width="414" height="351" alt="Screenshot 2026-03-20 184625" src="https://github.com/user-attachments/assets/14f1f518-6acc-4752-9b9e-e477a15cfc20" />](https://drive.google.com/file/d/1ayvyVb4PYAjlYvAcFlreRHea2IlMc_tX/view?usp=drive_link)

</div>

<hr>

## Autonomous Laps Videos 
<div align="center">
     
#### **[Real DonkeyCar 3 Autonomous Laps]**
[<img width="416" height="359" alt="Screenshot 2026-03-20 214816" src="https://github.com/user-attachments/assets/5dfbbc5f-c8b2-4af8-9304-6d8c184babf0" />](https://drive.google.com/file/d/1k6vuTRKD1uA1FilJbSSl9JWeUeOFdBSk/view)

</div>

<div align="center">

#### **[GPS 3 Autonomous Laps]**
[<img width="414" height="352" alt="Screenshot 2026-03-20 214537" src="https://github.com/user-attachments/assets/4d0d1677-4551-4711-8e1d-92527df9b849" />](https://drive.google.com/file/d/1TkWeKOp9QAR9elpqoll9n0WdSUxgWwWP/view?usp=drive_link)

</div>

<div align="center">

#### **[ROS2 3 Autonomous Laps]**
[<img width="414" height="352" alt="Screenshot 2026-03-20 214231" src="https://github.com/user-attachments/assets/442c28bd-0026-4777-b7f6-d377de814f54" />](https://drive.google.com/file/d/17f8Hzn6Xm5CPKD7XSKURB641qJMOlyrI/view?usp=drive_link)

</div>

<hr>

## CAD Designs for DonkeyCar 

__Parts List__
* [Main Board]
  ![Project Screenshot](https://cdn.discordapp.com/attachments/1457986642462769226/1484727828967854231/Screenshot_2026-03-20_183535.png?ex=69bf480b&is=69bdf68b&hm=41a86cc5967c6ba73cc0e7ad1f3d259fe6e608047ce8acb773b951c47b01687f&)
* [Camera Mount]
  ![Project Screenshot](https://cdn.discordapp.com/attachments/1457986642462769226/1484727831480111195/Screenshot_2026-03-20_183759.png?ex=69bf480b&is=69bdf68b&hm=f2ce9c1dcd6c87d6c288559c0b81ea5a0a242da425f56f49902df51ccf88975c&)
* [Lidar Mount]
  ![Project Screenshot](https://cdn.discordapp.com/attachments/1457986642462769226/1484727830951497769/Screenshot_2026-03-20_183720.png?ex=69bf480b&is=69bdf68b&hm=9da9fcd25316f9203ae6aba8728b08cb8d852d249d6317ae96d54ef6dbf9b1a1&)
* [All Components Combined]
  ![Project Screenshot](https://cdn.discordapp.com/attachments/1457986642462769226/1484727829366308874/Screenshot_2026-03-20_183557.png?ex=69bf480b&is=69bdf68b&hm=f4314217ef272e2d9a6878bfda306905eabc053d96c85370848032eb600ac2eb&)
  
<hr>

## Acknowledgements
Thanks to Professor Jack Silberman and both TAs Jose and Jingli for supporing our project during the course!

<hr>

## Contacts
* Spencer Herberich - sherberich@ucsd.edu 
* Pelin Kaynak - pkaynak@ucsd.edu
* Woojin Lee - wol022@ucsd.edu 
* Thomas Beauchataud - thomas.beauchataud@estaca.eu 

<hr>
