# Slødda 1 - MEPA2002 Mekatronikk 4

Beltedrevet redningsrobot med manipulator arm.

## Quick Start

### Forutsetninger
- Ubuntu 24.04 LTS
- ROS 2 Kilted
- Gazebo Harmonic

### Installasjon
```bash
cd ~
git clone https://github.com/DITTBRUKERNAVN/slodda_1.git
mkdir -p ~/slodda_ws/src
cd ~/slodda_ws/src
ln -s ~/slodda_1 .
cd ~/slodda_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Kjør Simulering
```bash
ros2 launch slodda_bringup sim.launch.py
```

## Struktur
```
slodda_1/
├── slodda_bringup/      # Launch files
├── slodda_description/  # URDF, meshes
├── slodda_gazebo/       # Gazebo worlds
├── firmware/            # Arduino/ESP32
├── docs/                # Dokumentasjon
└── scripts/             # Helper scripts
```

## Workflow

1. Lag issue på GitHub
2. `git checkout -b feature/min-feature`
3. `git commit -m "feat: beskrivelse"`
4. `git push origin feature/min-feature`
5. Lag Pull Request til dev

## Team

- Sigurd (tyngden)
- Jostein (slaktern)
- Isak (grøten)

## Kjør Simulering (detaljert)

### Forutsetninger
Installer Gazebo Harmonic separat hvis `gz sim` ikke finnes:
```bash
sudo apt install gazebo-harmonic
```

### Bygg og start
```bash
cd ~/slodda_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch slodda_gazebo gazebo.launch.py
```

### Styr roboten med tastatur
I ny terminal:
```bash
source ~/slodda_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Send kjørekommando manuelt
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --rate 10 --times 50
```

### Stopp roboten
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

## RPM-feedback fra motorer

Simuleringen publiserer hjulhastigheter via `JointStatePublisher`-pluginen i Gazebo,
bridget til ROS 2 på `/joint_states`. RPM beregnes fra posisjonsdifferansen over tid
siden Gazebo Harmonic ikke fyller ut velocity-feltet direkte.

### Tilgjengelige topics
| Topic | Type | Beskrivelse |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Kjørekommandoer inn |
| `/odom` | `nav_msgs/Odometry` | Odometri ut |
| `/joint_states` | `sensor_msgs/JointState` | Hjulposisjoner (rad) |
| `/ir_front_left` | `sensor_msgs/LaserScan` | IR-sensor venstre |
| `/ir_front_center` | `sensor_msgs/LaserScan` | IR-sensor senter |
| `/ir_front_right` | `sensor_msgs/LaserScan` | IR-sensor høyre |

### Overvåk RPM i sanntid
```bash
source ~/slodda_ws/install/setup.bash
python3 ~/slodda_ws/rpm_monitor.py
```

Forventet output ved full fart (0.3 m/s):
```
left_front_wheel_joint  :  -26.1 RPM
left_rear_wheel_joint   :  -26.1 RPM
right_front_wheel_joint :  +26.1 RPM
right_rear_wheel_joint  :  +26.1 RPM
```

Negativt fortegn på venstre hjul er korrekt — hjulene roterer speilvendt
for å drive roboten fremover.

### Motorspesifikasjon
- DFRobot FIT0450, 12V, maks 350 RPM
- Hjulradius: 45 mm
- Teoretisk topphastighet: ~1.65 m/s
