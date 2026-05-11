# Slødda 1 — MEPA2002 Mekatronikk 4

Autonom belterobot med ROS 2 Kilted, Nav2-navigasjon, YOLO-basert bjørn-deteksjon og TPU-kloarm.
Bygd på Raspberry Pi 4 med BNO055 IMU, LD06 lidar, DFR0601 motorer og ESP32-styrt arm.

> **Status:** Bekreftet stabil navigasjon og arm-kontroll på fysisk robot per 2026-05-11.
> Sim-stacken (Gazebo) er beholdt, men hovedfokus er hardware.

---

## Hurtiglenker

- [docs/CHEATSHEET.md](docs/CHEATSHEET.md) — kompakt kommandoreferanse
- [docs/RASPBERRY_PI_SETUP.md](docs/RASPBERRY_PI_SETUP.md) — full Pi-installasjon fra blank SD
- [firmware/README.md](firmware/README.md) — ESP32 flash-instruksjon

---

## 1. Hardware

| Komponent | Tilkobling | Detaljer |
|---|---|---|
| Raspberry Pi 4 (4 GB) | — | Ubuntu Server 24.04, ROS 2 Kilted |
| BNO055 IMU | I2C1 (GPIO 2/3), adresse `0x28` | Hardware I2C 400 kHz, `Adafruit_BNO055` |
| LD06 lidar | UART3 (GPIO 4/5), `/dev/ttyAMA3` | 230 400 baud |
| DFR0601 motorer | GPIO 18/19 PWM, 23/24/25/26 dir | 663 ticks/rev encoder |
| ESP32 (TPU-arm) | UART0 (GPIO 14/15), `/dev/ttyS0` | 115 200 baud, mini-UART |
| 3× HD W50-360 servo | ESP32 GPIO 27/26/25 | Kontinuerlige servoer, 7.4–8.1V |
| Sharp 2Y0A21 IR | ESP32 GPIO 35 | Auto-grip < 12 cm |

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
enable_uart=1
```

Reboot, deretter verifiser:
```bash
ls /dev/ttyAMA3        # skal finnes (lidar)
i2cdetect -y 1         # skal vise 0x28 (BNO055)
```

### 2.2 Aktiver UART til ESP32 (arm)

Pi sender boot-meldinger på GPIO14 (TX) som standard — dette gir falske servokommandoer.
Fjern serial-konsoll og deaktiver getty:

```bash
# Fjern console=serial0,115200 fra cmdline (én linje, ikke legg til linjeskift)
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt

# Deaktiver serial getty
sudo systemctl disable --now serial-getty@ttyS0.service

sudo reboot
```

Verifiser etter reboot:
```bash
cat /proc/cmdline | grep serial   # skal IKKE inneholde console=serial0
ls /dev/ttyS0                     # skal finnes
```

Kabling:
| Pi pin | Pi GPIO | → | ESP32 GPIO |
|--------|---------|---|-----------|
| Pin 8 (TXD) | GPIO 14 | → | GPIO 13 (RX2) |
| Pin 10 (RXD) | GPIO 15 | ← | GPIO 17 (TX2) |
| Pin 6/14 (GND) | GND | — | GND |

### 2.3 Python-biblioteker

```bash
pip install Adafruit_BNO055 pyserial --break-system-packages
```

### 2.4 ROS 2-avhengigheter

```bash
sudo apt install -y \
  ros-kilted-xacro ros-kilted-robot-localization \
  ros-kilted-nav2-behaviors ros-kilted-nav2-bt-navigator \
  ros-kilted-nav2-controller ros-kilted-nav2-lifecycle-manager \
  ros-kilted-nav2-planner ros-kilted-nav2-smoother \
  ros-kilted-robot-state-publisher ros-kilted-tf2-ros \
  i2c-tools
```

### 2.5 Klon og bygg

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

Oppstartsfaser (~35 s totalt):

| Tid | Hva starter |
|-----|-------------|
| 0 s | robot_state_publisher, static map→odom TF, lidar, IMU, arm_controller, motor_driver |
| 4 s | odometry_node, EKF |
| 12 s | controller_server, planner_server |
| 18 s | smoother_server, behavior_server, bt_navigator |
| 30 s | lifecycle_manager → venter på `Managed nodes are active` |

`arm_controller_node` starter i fase 0 og homer armen til posisjon 0 automatisk.

### Laptop — start RViz

```bash
ros2 run rviz2 rviz2 -d ~/slodda_1/src/slodda_bringup/config/hardware.rviz
```

### Arm-kontroll

```bash
# Sett arm i kjøreposisjon (gjør dette etter launch)
ros2 service call /arm/drive  std_srvs/srv/Trigger {}

# Søkemodus — aktiverer IR auto-grip
ros2 service call /arm/search std_srvs/srv/Trigger {}

# Tilbake til hvile
ros2 service call /arm/open   std_srvs/srv/Trigger {}

# Nødstopp
ros2 service call /arm/stop   std_srvs/srv/Trigger {}

# Overvåk arm-status
ros2 topic echo /arm/status
```

### Manuell kalibrering av armposisjoner

Krever at `arm_controller_node` kjører:
```bash
python3 ~/slodda_1/src/slodda_bringup/scripts/arm_manual_control.py
```

Lagre posisjoner med `z/x/c/v` (pos 0/1/2/3).

### Navigasjonsmål

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Eller bruk **2D Nav Goal** i RViz.

---

## 5. TPU-arm

### Posisjoner

| Posisjon | Navn | Beskrivelse |
|----------|------|-------------|
| 0 | OPEN | Hvile/åpen — standard ved oppstart og avslutning |
| 1 | DRIVE | Arm trukket inn for trygg navigasjon |
| 2 | SEARCH | Arm strukket ut, IR-overvåking aktiv |
| 3 | GRAB | Arm lukket rundt objekt |

### Auto-grip

Når armen er i posisjon 2 og IR-sensor ser objekt < 12 cm i 0.75 s:
firmware går automatisk `pos 0 → pos 3` og publiserer `GRABBED` på `/arm/status`.

### ROS2-grensesnitt

| Topic/Service | Type | Beskrivelse |
|---|---|---|
| `/arm/open` | `std_srvs/Trigger` | Gå til pos 0 |
| `/arm/drive` | `std_srvs/Trigger` | Gå til pos 1 |
| `/arm/search` | `std_srvs/Trigger` | Gå til pos 2, IR aktiveres |
| `/arm/grab` | `std_srvs/Trigger` | Gå til pos 3 |
| `/arm/stop` | `std_srvs/Trigger` | Nødstopp |
| `/arm/status` | `std_msgs/String` | ESP32-feedback: `DONE:X`, `GRABBED`, `STOPPED` |
| `/arm/raw_command` | `std_msgs/String` | Send enkeltbyte direkte til ESP32 |

Hardware-panelet (`ros2 run slodda_bringup hardware_panel`) har en **ARM STOPP**-knapp.

---

## 6. EKF sensor fusion

| Kilde | Topic | Bidrag |
|---|---|---|
| Encoder odometry | `/odom` | vx (fremover-hastighet) |
| BNO055 gyro | `/imu/data` | vyaw (yaw-rate) |

Encoder-yaw er deaktivert — belter glir ved rotasjon og gir unøyaktig yaw.
IMU gyro er eneste yaw-rate-kilde. EKF publiserer `odom→base_footprint` TF.

---

## 7. Kjente problemer

| Symptom | Årsak | Fiks |
|---|---|---|
| Servoer gjør random bevegelser ved oppstart | `console=serial0` i cmdline.txt sender boot-output på GPIO14 | Kjør sed-kommandoen i seksjon 2.2 og reboot |
| `Kan ikke åpne /dev/ttyS0: Permission denied` | serial-getty holder porten | `sudo systemctl disable --now serial-getty@ttyS0` + `sudo chmod 666 /dev/ttyS0` |
| Arm TIMEOUT på service-kall | ESP32 svarer ikke — feil pins eller firmware ikke lastet | Verifiser GPIO13/17-kabling og at firmware 66 er flashet |
| `BNO055 init failed` | I2C ikke aktivert | `i2cdetect -y 1` → sjekk `0x28`; verifiser `dtparam=i2c_arm=on` |
| Nav2 aktiverer aldri | CPU overload eller bond-timeout | Vent 60 s; `htop`; reboot Pi |
| Spin gjør for mange runder | Yaw-konflikt encoder/IMU | Bekreft `odom0_config` har vyaw=false i `ekf.yaml` |
| Topics ikke synlig fra laptop | DOMAIN_ID feil | `echo $ROS_DOMAIN_ID` → skal vise `42` på begge |
| Launch-endringer virker ikke | data_files er kopier | `colcon build --packages-select slodda_bringup` |

---

## 8. Repo-struktur

```
slodda_1/
├── firmware/
│   ├── servo_tid_test66.ino   # ESP32 arm-firmware (Serial2-only)
│   └── README.md              # Flash-instruksjon og pin-mapping
├── src/
│   ├── slodda_bringup/        # Launch, Nav2-config, Python-noder (HOVEDPAKKE)
│   │   ├── slodda_bringup/
│   │   │   ├── arm_controller_node.py   # ROS2 ↔ ESP32 UART-bro
│   │   │   └── ...
│   │   └── scripts/
│   │       └── arm_manual_control.py    # Manuell kalibrering
│   ├── slodda_description/    # URDF/Xacro robot-modell
│   ├── slodda_gazebo/         # Simulasjon (Gazebo Harmonic)
│   ├── slodda_vision/         # Kamera og YOLO-deteksjon
│   └── ldlidar_stl_ros2/      # LD06 lidar-driver
├── docs/
│   ├── CHEATSHEET.md          # Kompakt kommandoreferanse
│   └── RASPBERRY_PI_SETUP.md  # Full Pi-installasjon
└── README.md
```
