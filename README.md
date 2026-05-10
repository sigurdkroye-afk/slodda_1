# Slødda 1 — MEPA2002 Mekatronikk 4

Autonom belterobot med ROS 2 Kilted, Nav2-navigasjon og YOLO-basert bjørn-deteksjon.
Bygd på Raspberry Pi 4 med BNO055 IMU, LD06 lidar og DFR0601 motorer.

> **Status:** Bekreftet stabil navigasjon på fysisk robot per 2026-05-10.
> Sim-stacken (Gazebo) er beholdt, men hovedfokus er hardware.

---

## Hurtiglenker

- [docs/CHEATSHEET.md](docs/CHEATSHEET.md) — kompakt kommandoreferanse
- [docs/RASPBERRY_PI_SETUP.md](docs/RASPBERRY_PI_SETUP.md) — full Pi-installasjon fra blank SD

---

## 1. Hardware

| Komponent | Tilkobling | Detaljer |
|---|---|---|
| Raspberry Pi 4 (4 GB) | — | Ubuntu Server 24.04, ROS 2 Kilted |
| BNO055 IMU | I2C1 (GPIO 2/3), adresse `0x28` | Hardware I2C 400 kHz, `Adafruit_BNO055` |
| LD06 lidar | UART3 (GPIO 4/5), `/dev/ttyAMA3` | 230 400 baud |
| DFR0601 motorer | GPIO 18/19 PWM, 23/24/25/26 dir | 663 ticks/rev encoder |

Wheelbase: 0.2316 m — Hjulradius: 0.01021 m (kalibrert mot målt distanse)

---

## 2. Pi-oppsett

> Antar Ubuntu Server 24.04 og ROS 2 Kilted er installert.
> Full installasjon fra blank SD: se `docs/RASPBERRY_PI_SETUP.md`.

### 2.1 Aktiver hardware I2C og UART3

```bash
sudo nano /boot/firmware/config.txt
```

Legg til / endre:
```
dtoverlay=uart3
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

Reboot, deretter verifiser:
```bash
ls /dev/ttyAMA3        # skal finnes
i2cdetect -y 1         # skal vise 0x28 (BNO055)
```

### 2.2 Python-biblioteker

```bash
pip install Adafruit_BNO055 --break-system-packages
```

### 2.3 ROS 2-avhengigheter

```bash
sudo apt install -y \
  ros-kilted-xacro ros-kilted-robot-localization \
  ros-kilted-nav2-behaviors ros-kilted-nav2-bt-navigator \
  ros-kilted-nav2-controller ros-kilted-nav2-lifecycle-manager \
  ros-kilted-nav2-planner ros-kilted-nav2-smoother \
  ros-kilted-robot-state-publisher ros-kilted-tf2-ros \
  i2c-tools
```

### 2.4 Klon og bygg

```bash
git clone https://github.com/sigurdkroye-afk/slodda_1.git ~/slodda_1
cd ~/slodda_1
colcon build --symlink-install
source install/setup.bash
```

Legg til i `~/.bashrc`:
```bash
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash
export ROS_DOMAIN_ID=42
```

### ⚠️ data_files-gotcha

`slodda_bringup` er `ament_python` — launch-filer og YAML-konfig er **kopier** i `install/`.
Etter `git pull` på Pi **alltid** kjør:

```bash
colcon build --packages-select slodda_bringup
```

---

## 3. Laptop-oppsett

```bash
git clone https://github.com/sigurdkroye-afk/slodda_1.git ~/slodda_1
cd ~/slodda_1 && colcon build --symlink-install
```

Legg til i `~/.bashrc`:
```bash
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash
export ROS_DOMAIN_ID=42
```

Begge maskiner må være på samme nettverk. Tailscale fungerer (Pi: `100.66.136.28`).

---

## 4. Kjøring

### Pi — start full stack

```bash
ssh slodda1@sloddapi
ros2 launch slodda_bringup hardware.launch.py
```

Oppstartsfaser (~30 s totalt):

| Tid | Hva starter |
|-----|-------------|
| 0 s | robot_state_publisher, static map→odom TF, lidar, IMU, motor_driver |
| 4 s | odometry_node, EKF |
| 12 s | controller_server, planner_server |
| 18 s | smoother_server, behavior_server, bt_navigator |
| 30 s | lifecycle_manager → venter på `Managed nodes are active` |

### Laptop — start RViz

```bash
ros2 run rviz2 rviz2 -d ~/slodda_1/src/slodda_bringup/config/hardware.rviz
```

### Verifiser at alt kjører

```bash
ros2 topic hz /scan              # ~10 Hz
ros2 topic hz /imu/data          # ~50 Hz
ros2 topic hz /odometry/filtered # ~12 Hz
```

### Spin-test (360° på stedet)

```bash
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 6.28}"
```

### Navigasjonsmål

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Eller bruk **2D Nav Goal** i RViz.

---

## 5. EKF sensor fusion

| Kilde | Topic | Bidrag |
|---|---|---|
| Encoder odometry | `/odom` | vx (fremover-hastighet) |
| BNO055 gyro | `/imu/data` | vyaw (yaw-rate) |

Encoder-yaw er deaktivert — belter glir ved rotasjon og gir unøyaktig yaw.
IMU gyro er eneste yaw-rate-kilde. EKF publiserer `odom→base_footprint` TF.

---

## 6. Kjente problemer

| Symptom | Årsak | Fiks |
|---|---|---|
| `BNO055 init failed` | I2C ikke aktivert | `i2cdetect -y 1` → sjekk `0x28`; verifiser `dtparam=i2c_arm=on` |
| `Adafruit_BNO055 not installed` | Bibliotek mangler | `pip install Adafruit_BNO055 --break-system-packages` |
| Nav2 aktiverer aldri | CPU overload eller bond-timeout | Vent 60 s; `htop`; reboot Pi |
| Spin gjør for mange runder | Yaw-konflikt encoder/IMU | Bekreft `odom0_config` har vyaw=false i `ekf.yaml` |
| Topics ikke synlig fra laptop | DOMAIN_ID feil | `echo $ROS_DOMAIN_ID` → skal vise `42` på begge |
| Launch-endringer virker ikke | data_files er kopier | `colcon build --packages-select slodda_bringup` |

---

## 7. Repo-struktur

```
slodda_1/
├── src/
│   ├── slodda_bringup/        # Launch, Nav2-config, Python-noder (HOVEDPAKKE)
│   ├── slodda_description/    # URDF/Xacro robot-modell
│   ├── slodda_gazebo/         # Simulasjon (Gazebo Harmonic)
│   ├── slodda_vision/         # Kamera og YOLO-deteksjon
│   └── ldlidar_stl_ros2/      # LD06 lidar-driver
├── docs/
│   ├── CHEATSHEET.md          # Kompakt kommandoreferanse
│   └── RASPBERRY_PI_SETUP.md  # Full Pi-installasjon
└── README.md
```
