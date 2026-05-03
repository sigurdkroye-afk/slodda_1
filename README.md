# Slødda 1 — MEPA2002 Mekatronikk 4

Autonomous tracked rescue robot with Nav2 navigation, YOLO-based object detection, and a full bear-mission state machine. Built with ROS 2 Kilted + Gazebo Harmonic.

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        MISSION CONTROL                          │
│                                                                 │
│  IDLE ──► NAVIGATE ──► CANCELING ──► TRACK_BEAR ──► AT_BEAR    │
│    ▲          │                           │            │        │
│    │          │   (bear spotted)          │ (aligned)  │        │
│    └──────────┴───────────────────────────┴────────────┘        │
│                                                APPROVE ▼        │
│                                           RETURN_HOME ──► IDLE  │
└─────────────────────────────────────────────────────────────────┘
```

| State | Description |
|---|---|
| `IDLE` | Waiting for a Nav2 `2D Goal Pose` from RViz. YOLO disarmed. |
| `NAVIGATE` | Nav2 drives toward the goal. YOLO armed — triggers override if bear visible. |
| `CANCELING` | 1.5 s zero-velocity hold after cancelling Nav2 goal — ensures `controller_server` stops. |
| `TRACK_BEAR` | Camera + LiDAR P-controller: rotate to centre bear, then approach to 20 cm. |
| `AT_BEAR` | Stopped at bear, waiting for `APPROVE` command before returning. |
| `RETURN_HOME` | Replays the recorded odometry path in reverse at 15 cm/s. YOLO disarmed. |

### YOLO-triggered navigation override

While in `NAVIGATE`, the YOLO detector runs at 5 Hz on class 77 (teddy bear). Three consecutive detections arm the override. Nav2's goal is cancelled, the robot holds zero velocity for 1.5 s (so Nav2's `controller_server` drains its `/cmd_vel` output), then transitions directly to `TRACK_BEAR`.

### Tracking controller (TRACK_BEAR)

Two-phase approach:
1. **Rotate** — zero linear velocity, proportional angular control on pixel error until bear is centred (< 20 px error).
2. **Approach** — proportional linear control on LiDAR distance to 20 cm target. Drift correction re-centres bear if pixel error exceeds 30 px. LiDAR readings below 5 cm trigger immediate `AT_BEAR`. Fallback: if LiDAR returns `inf` (bear beyond 2 m), creep forward at 5 cm/s while maintaining alignment.

### Return-home path replay

During `NAVIGATE`, `CANCELING`, and `TRACK_BEAR`, the robot records odometry waypoints every 5 cm (up to 500 points). On `RETURN_HOME`, this path is reversed and replayed at 15 cm/s with proportional angular correction toward each waypoint heading.

---

## Packages

| Package | Description |
|---|---|
| `slodda_bringup` | All Python nodes, Nav2 config, launch files |
| `slodda_description` | URDF/Xacro robot model, RViz config |
| `slodda_gazebo` | Gazebo world, arena map, sim launch |
| `slodda_vision` | Camera node (real robot) |
| `slodda_rviz_panel` | Custom RViz control panel plugin |

### Key nodes

| Node | Executable | Description |
|---|---|---|
| Mission control | `mission_control` | Main state machine (Nav2 + tracking + return-home) |
| YOLO detector | `yolo_detector` | YOLOv8n on `/camera/image_raw`, publishes `/yolo/detections` |
| Object tracker | `object_tracker` | Color-based (red) fallback tracker |
| Motor driver | `motor_driver` | `/cmd_vel` → PWM on real robot (configure GPIO pins before use) |
| A* planner | `astar_planner` | Custom grid planner |
| APF controller | `apf_controller` | Artificial potential field controller |

---

## ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (IN) |
| `/odom` | `nav_msgs/Odometry` | Odometry (OUT) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR — LD06 (OUT) |
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera (OUT) |
| `/yolo/detections` | `vision_msgs/Detection2DArray` | YOLO bear detections (OUT) |
| `/yolo/image` | `sensor_msgs/Image` | Annotated camera image (OUT) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 2D Nav goal from RViz (IN) |
| `/mission/cmd` | `std_msgs/String` | Mission commands: `APPROVE`, `CANCEL`, `CANCEL_RETURN` (IN) |
| `/mission/status` | `std_msgs/String` | Current state, published at 10 Hz (OUT) |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint positions (OUT) |
| `/imu/data` | `sensor_msgs/Imu` | IMU (OUT) |
| `/ir_front_left/center/right` | `sensor_msgs/LaserScan` | IR distance sensors (OUT) |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree (OUT) |

---

## Quick Start — Simulation

**Prerequisites:** Ubuntu 24.04 LTS, ROS 2 Kilted, Gazebo Harmonic.

### 1. Clone and build

```bash
git clone https://github.com/sigurdkroye-afk/slodda_1.git ~/slodda_1
cd ~/slodda_1
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add convenience aliases to `~/.bashrc`:

```bash
echo "source /opt/ros/kilted/setup.bash"           >> ~/.bashrc
echo "source ~/slodda_1/install/setup.bash"        >> ~/.bashrc
echo "alias cb='cd ~/slodda_1 && colcon build --symlink-install'" >> ~/.bashrc
echo "alias cs='source ~/slodda_1/install/setup.bash'"            >> ~/.bashrc
echo "alias cw='cd ~/slodda_1'"                    >> ~/.bashrc
source ~/.bashrc
```

### 2. Launch the full simulation stack

```bash
# Terminal 1 — Gazebo + Nav2 + RViz + YOLO + mission_control
ros2 launch slodda_bringup sim.launch.py
```

Wait ~30 s for Nav2 lifecycle nodes to activate (the costmap appears in RViz when ready).

### 3. Run a mission

Use the **2D Goal Pose** button in RViz to set a navigation target. The robot will drive to the goal. If it spots the bear while navigating, it overrides Nav2 and approaches automatically.

```bash
# Approve at bear (after robot stops at bear):
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: 'APPROVE'"

# Cancel return home (emergency):
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: 'CANCEL_RETURN'"

# Watch mission state:
ros2 topic echo /mission/status
```

### 4. Manual control

```bash
# Keyboard teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Stop robot:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Quick Start — Real Robot (Raspberry Pi)

See [`docs/RASPBERRY_PI_SETUP.md`](docs/RASPBERRY_PI_SETUP.md) for full setup. See [`docs/CHEATSHEET.md`](docs/CHEATSHEET.md) for GPIO pin mapping and first-time Pi setup.

### Hardware

| Sensor/Aktuator | Tilkobling |
|---|---|
| Motorer (DFR0601) | GPIO 18/19 (PWM), 23/24 (venstre dir), 25/26 (høyre dir) |
| LiDAR LD06 | GPIO 4/5 → UART3 (`/dev/ttyAMA3`) |
| IMU BNO085 | GPIO 2/3 → I2C1, adresse `0x4a` |

### Launch

```bash
# SSH:
ssh slodda1@SloddaPi.local

# Full stack:
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash
ros2 launch slodda_bringup hardware.launch.py

# Bare motorer (testing):
ros2 launch slodda_bringup motors_only.launch.py

# On your laptop (same WiFi, same ROS_DOMAIN_ID):
export ROS_DOMAIN_ID=42
ros2 launch slodda_description view_real_robot.launch.py
```

---

## Repository Layout

```
slodda_1/
├── src/
│   ├── slodda_bringup/         # Python nodes, Nav2 config, launch files
│   ├── slodda_description/     # URDF, RViz config
│   ├── slodda_gazebo/          # Gazebo world, arena map
│   ├── slodda_vision/          # Camera node (real robot)
│   └── slodda_rviz_panel/      # Custom RViz panel
├── docs/
│   ├── RASPBERRY_PI_SETUP.md   # Full Pi setup guide
│   ├── CHEATSHEET.md           # ROS2 / Git quick reference
│   └── guides/                 # Troubleshooting, setup guides
├── firmware/                   # ESP32 / Arduino code
└── scripts/                    # Utility scripts
```

---

## Team — NTNU MEPA2002 2026

- Sigurd Kristian Øye
- Jostein
- Isak
