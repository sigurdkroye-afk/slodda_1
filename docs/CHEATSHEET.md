# Slødda 1 — Cheatsheet

Workspace: `~/slodda_1` | ROS: Kilted | `ROS_DOMAIN_ID=42` | Pi: `slodda1@sloddapi` / `100.66.136.28`

---

## SSH til Pi

```bash
ssh slodda1@sloddapi
# eller via Tailscale-IP:
ssh slodda1@100.66.136.28
```

På begge maskiner:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash
```

---

## Launch

På Pi:

```bash
ros2 launch slodda_bringup hardware.launch.py       # full hardware-stack (~35s oppstart)
ros2 launch slodda_bringup mission_full.launch.py   # hardware + kamera + yolo + bear_mission
```

Med mission-parametere:

```bash
ros2 launch slodda_bringup mission_full.launch.py goal_x:=1.0 goal_y:=0.0 grab_timeout_s:=30
```

På laptop (RViz):

```bash
ros2 run rviz2 rviz2 -d ~/slodda_1/src/slodda_bringup/config/hardware.rviz
```

---

## Hva launcher hva

**`hardware.launch.py`**
- `robot_state_publisher`, statiske TF-er
- LiDAR (LD06), IMU (BNO055)
- `arm_controller`, `motor_driver`
- Encoder-odometri, EKF
- Nav2: `controller_server`, `planner_server`, `smoother_server`, `behavior_server`, `bt_navigator`
- `lifecycle_manager`

**`mission_full.launch.py`**
- Inkluderer hele `hardware.launch.py`
- Ved t=35s: `camera_node`, `yolo_detector`, `bear_mission`

---

## Bear Mission

Trigger:

```bash
ros2 topic pub --once /bear_mission/start std_msgs/msg/Bool "data: true"
```

States (rekkefølge):

```
IDLE -> NAVIGATE_TO_GOAL -> CANCELING -> VERIFY_BEAR -> VISUAL_SERVO -> RETURN_HOME -> FINAL_RELEASE
```

Snarvei: hvis bjørn er synlig ved start hoppes Nav2 over og maskinen går rett til `VERIFY_BEAR`.

Overvåk:

```bash
ros2 topic echo /bear_mission/status
bash ~/slodda_1/src/slodda_bringup/scripts/monitor_mission.sh
```

Parametere ved launch: `goal_x`, `goal_y`, `grab_timeout_s` (default 30s)

---

## Arm-kontroll

Posisjoner: **0=OPEN** | **1=DRIVE** | **2=SEARCH (IR aktiv)** | **3=GRAB**

```bash
ros2 service call /arm/open   std_srvs/srv/Trigger {}   # pos 0 — hvile
ros2 service call /arm/drive  std_srvs/srv/Trigger {}   # pos 1 — kjøring
ros2 service call /arm/search std_srvs/srv/Trigger {}   # pos 2 — søk + IR auto-grip
ros2 service call /arm/grab   std_srvs/srv/Trigger {}   # pos 3 — grep
ros2 service call /arm/stop   std_srvs/srv/Trigger {}   # nødstopp

ros2 topic echo /arm/status                              # DONE:X / GRABBED / STOPPED
```

Manuell kalibrering:

```bash
python3 ~/slodda_1/src/slodda_bringup/scripts/arm_manual_control.py
# t/g/y/h/u/j = servo-steg | z/x/c/v = lagre pos 0/1/2/3 | q = nødstopp
```

---

## Topics — forventede rater

```bash
ros2 topic hz /odometry/filtered     # ~8 Hz
ros2 topic hz /imu/data              # ~10 Hz
ros2 topic hz /odom                  # encoder odometri
ros2 topic hz /scan                  # ~10 Hz (LiDAR)
ros2 topic hz /cmd_vel
ros2 topic hz /camera/image_raw      # ~5 Hz (kun om camera_node kjører)
```

---

## TF og lifecycle

```bash
ros2 run tf2_tools view_frames                   # genererer PDF av TF-tre
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom

ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 node list | grep -E 'nav2|ekf|mission'
```

---

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"   # stopp
```

---

## Debugging

```bash
top -bn1 | head -20                  # CPU på Pi
htop
journalctl -u serial-getty@ttyS0 -n 50
dmesg | tail -30
i2cdetect -y 1                       # BNO055 skal vises på 0x28
ls -l /dev/ttyS0 /dev/ttyAMA3
```

DDS-opprydding:

```bash
bash ~/slodda_1/src/slodda_bringup/scripts/clean_dds.sh
```

---

## Bag-recording

```bash
ros2 bag record /odometry/filtered /imu/data /scan /tf /tf_static /cmd_vel /arm/status /yolo/detections
ros2 bag record -a                   # alt (mye plass)
ros2 bag play <bag-dir>
ros2 bag info <bag-dir>
```

---

## Build (lokalt)

```bash
cd ~/slodda_1
colcon build --symlink-install
source install/setup.bash
```

---

## Deploy til Pi

Fra laptop:

```bash
git push
```

På Pi:

```bash
cd ~/slodda_1
git pull
bash src/slodda_bringup/scripts/install.sh
```

`install.sh` kopierer launch/yaml/Python til `install/` — ingen colcon rebuild nødvendig etter kode- og config-endringer.

---

## Git

```bash
git status
git pull --rebase
git push
git log --oneline -10
```

---

## Gotchas

| Problem | Fiks |
|---|---|
| Servoer gjør random bevegelser ved oppstart | `console=serial0` i `cmdline.txt` — kjør sed-kommando i README seksjon 2.2 + reboot |
| `/dev/ttyS0: Permission denied` | `sudo systemctl disable --now serial-getty@ttyS0` |
| Arm TIMEOUT | Verifiser GPIO13 (RX) og GPIO17 (TX) til ESP32 — ikke GPIO16 (ustabil) |
| Launch-endringer virker ikke etter `git pull` | Bruk `install.sh`, ikke `colcon build` |
| Topics ikke synlig fra laptop | `export ROS_DOMAIN_ID=42` på begge maskiner |
| BNO055 ikke funnet | `i2cdetect -y 1` → sjekk `0x28`; `dtparam=i2c_arm=on` i `config.txt` |
| `StopIteration` ved oppstart av Python-noder | egg-info korrupt — kjør `colcon build --packages-select slodda_bringup` |
| Nav2 starter ikke / lifecycle timeout | Vent 60s; `htop`; reboot Pi om CPU >90% |
| EKF "Failed to meet update rate" | Normalt på Pi4 med full stack — OK så lenge `/odometry/filtered` er ~8 Hz |
| Roboten spinner under kjøring | EKF-frekvens for lav vs Nav2 controller — sjekk `ekf.yaml` `frequency` |
| Zombie DDS-participants etter kræsj | `bash src/slodda_bringup/scripts/clean_dds.sh` |

---

## Nødstopp

```bash
# Stopp launch:
Ctrl+C i launch-terminal

# Send nullhastighet:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# Hard: trekk strøm til motorer
```
