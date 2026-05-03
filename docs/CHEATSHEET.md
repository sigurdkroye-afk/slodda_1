# MEKATRONIKK 4 — ROS2 / GIT COMMANDS

Quick reference for the Slødda 1 project.
Workspace root: `~/slodda_1` | ROS distro: `kilted` | Domain ID: `42`

---

## Environment Setup

```bash
# Source ROS and workspace (run once per terminal, or add to ~/.bashrc)
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash

# Required env vars (same on all machines that communicate)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Add everything to `~/.bashrc` so it loads automatically:

```bash
echo "source /opt/ros/kilted/setup.bash"           >> ~/.bashrc
echo "source ~/slodda_1/install/setup.bash"        >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42"                     >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"  >> ~/.bashrc
echo "alias cb='cd ~/slodda_1 && colcon build --symlink-install'" >> ~/.bashrc
echo "alias cs='source ~/slodda_1/install/setup.bash'"            >> ~/.bashrc
echo "alias cw='cd ~/slodda_1'"                    >> ~/.bashrc
source ~/.bashrc
```

---

## Build Shortcuts

| Command | What it does |
|---|---|
| `cb` | Build all packages (`colcon build --symlink-install`) |
| `cs` | Source the workspace |
| `cw` | `cd ~/slodda_1` |

```bash
# Build all packages
cd ~/slodda_1 && colcon build --symlink-install

# Build one package only
colcon build --packages-select slodda_bringup --symlink-install

# Clean build (if things break)
cd ~/slodda_1
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## Simulation

```bash
# Full stack: Gazebo + Nav2 + RViz + YOLO + mission_control
ros2 launch slodda_bringup sim.launch.py

# Gazebo only (no Nav2):
ros2 launch slodda_gazebo gazebo.launch.py

# Keyboard teleop (new terminal):
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Send a single velocity command:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Stop the robot:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Mission Control

```bash
# Watch mission state in real time:
ros2 topic echo /mission/status

# Approve at bear (triggers RETURN_HOME):
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: 'APPROVE'"

# Cancel return home (emergency stop):
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: 'CANCEL_RETURN'"

# Cancel current mission:
ros2 topic pub --once /mission/cmd std_msgs/msg/String "data: 'CANCEL'"
```

Mission states in order:
```
IDLE → NAVIGATE → [CANCELING] → TRACK_BEAR → AT_BEAR → RETURN_HOME → IDLE
```

---

## Topics / Debugging

```bash
# List all active topics:
ros2 topic list

# Show message rate and type:
ros2 topic info /scan
ros2 topic hz /scan

# Echo a topic:
ros2 topic echo /odom
ros2 topic echo /mission/status
ros2 topic echo /yolo/detections

# List all running nodes:
ros2 node list

# Inspect a node:
ros2 node info /mission_control

# TF tree (save to PDF):
ros2 run tf2_tools view_frames
```

---

## RViz

```bash
# Launch RViz with Nav2 config:
ros2 run rviz2 rviz2 -d ~/slodda_1/src/slodda_bringup/config/nav2.rviz

# Set a 2D Nav Goal from RViz:
# Use the "2D Goal Pose" button in the toolbar (or press G)
```

Useful RViz displays to add manually:
- **Image** → Topic: `/yolo/image` (annotated camera with detections)
- **LaserScan** → Topic: `/scan`
- **Odometry** → Topic: `/odom`
- **Map** → Topic: `/map`

---

## Custom Nodes

```bash
# A* planner:
ros2 run slodda_bringup astar_planner

# APF (Artificial Potential Field) controller:
ros2 run slodda_bringup apf_controller

# YOLO detector (standalone):
ros2 run slodda_bringup yolo_detector

# Object tracker (color-based, red):
ros2 run slodda_bringup object_tracker

# Motor driver (real robot only — configure GPIO pins first):
ros2 run slodda_bringup motor_driver
```

---

## Camera — Pi to Laptop

**On the Raspberry Pi:**
```bash
source ~/slodda_1/install/setup.bash
ros2 launch slodda_vision camera.launch.py
```

**On the laptop** (same WiFi, `ROS_DOMAIN_ID=42`):
```bash
export ROS_DOMAIN_ID=42
ros2 topic echo /camera/image_raw   # verify stream is arriving

# View in RViz: Add → Image → Topic: /camera/image_raw
# Or view with:
ros2 run rqt_image_view rqt_image_view
```

If topics from Pi are not visible on laptop:
```bash
# Confirm domain IDs match:
echo $ROS_DOMAIN_ID   # must be 42 on both machines

# Check both machines are on the same subnet:
ping SloddaPi.local
```

---

## Raspberry Pi

### SSH

```bash
# By hostname (works on same WiFi):
ssh slodda1@SloddaPi.local

# By IP (if hostname doesn't resolve):
ssh slodda1@<IP_ADDRESS>
```

### File transfer (SCP)

```bash
# Copy file from laptop to Pi:
scp myfile.py slodda1@SloddaPi.local:~/slodda_1/src/slodda_bringup/slodda_bringup/

# Copy file from Pi to laptop:
scp slodda1@SloddaPi.local:~/slodda_1/somefile.py .

# Copy entire folder:
scp -r local_folder/ slodda1@SloddaPi.local:~/destination/
```

### Pi system commands

```bash
# Safe shutdown:
sudo shutdown now

# Reboot:
sudo reboot -f

# Check CPU/memory:
htop

# Check available disk:
df -h
```

---

## Real Robot Launch

```bash
# SSH inn på Pi:
ssh slodda1@SloddaPi.local

# Full hardware stack (Nav2 + LiDAR + IMU + motorer):
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash
ros2 launch slodda_bringup hardware.launch.py

# Bare motorer (for testing uten Nav2):
ros2 launch slodda_bringup motors_only.launch.py

# Keyboard control (på din maskin):
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Vent på: `[lifecycle_manager_navigation]: Managed nodes are active` (~3 min på Pi)

---

## Hardware — GPIO Pin Mapping (DFR0601)

| Signal | GPIO (BCM) | DFR0601 pin |
|---|---|---|
| Venstre motor PWM | 18 | P1 |
| Venstre motor dir A | 23 | A1 |
| Venstre motor dir B | 24 | B1 |
| Høyre motor PWM | 19 | P2 |
| Høyre motor dir A | 26 | B2 (invertert) |
| Høyre motor dir B | 25 | A2 |

LiDAR (LD06): GPIO4=TX, GPIO5=RX → `/dev/ttyAMA3`
IMU (BNO085): GPIO2=SDA, GPIO3=SCL → I2C1, adresse `0x4a`

---

## Pi — Første gangs oppsett (én gang)

```bash
# Aktiver UART3 (LiDAR) og I2C (IMU):
echo "dtoverlay=uart3" | sudo tee -a /boot/firmware/config.txt
echo "dtparam=i2c_arm=on" | sudo tee -a /boot/firmware/config.txt
sudo reboot

# Verifiser etter reboot:
ls /dev/ttyAMA3        # LiDAR port
i2cdetect -y 1         # skal vise 0x4a (IMU)

# Installer Python-avhengigheter:
pip install lgpio adafruit-blinka adafruit-circuitpython-bno08x --break-system-packages

# Installer ROS-avhengigheter:
sudo apt install -y ros-kilted-xacro ros-kilted-robot-localization \
  ros-kilted-nav2-amcl ros-kilted-nav2-behaviors ros-kilted-nav2-bt-navigator \
  ros-kilted-nav2-controller ros-kilted-nav2-lifecycle-manager \
  ros-kilted-nav2-map-server ros-kilted-nav2-planner ros-kilted-nav2-smoother \
  ros-kilted-robot-state-publisher i2c-tools

# Bygg workspace (ekskluder desktop-pakker):
cd ~/slodda_1 && git pull
colcon build --symlink-install --packages-ignore slodda_rviz_panel slodda_vision
```

---

## Git Workflow

### Daily workflow

```bash
# Always pull latest before starting:
git checkout dev
git pull origin dev

# Create a feature branch:
git checkout -b feat/my-feature

# Stage and commit:
git add src/slodda_bringup/slodda_bringup/myfile.py
git commit -m "feat: describe what and why"

# Push branch:
git push origin feat/my-feature
# → Open Pull Request on GitHub: feat/my-feature → dev
```

### Merge dev → main (release)

```bash
git checkout main
git pull origin main
git merge dev
git push origin main
git checkout dev
```

### Commit message format

```
feat:     new feature
fix:      bug fix
refactor: restructuring without behaviour change
docs:     documentation only
test:     add or fix tests
chore:    build, deps, config
```

Examples:
```bash
git commit -m "feat: YOLO-triggered Nav2 override with bear tracking"
git commit -m "fix: LiDAR fallback now stops when too close instead of creeping"
git commit -m "refactor: extract _enter_approach_controller helper"
```

### Useful git commands

```bash
git status                        # what's changed
git log --oneline -10             # recent commits
git diff                          # unstaged changes
git diff --staged                 # staged changes
git stash                         # save dirty state temporarily
git stash pop                     # restore stashed state
git checkout -- <file>            # discard changes to one file
git branch -a                     # list all branches
git fetch --all                   # sync remote without merging
```

---

## Odometry / Sensor Topics

```bash
# Live odometry pose:
ros2 topic echo /odom

# LiDAR scan:
ros2 topic echo /scan

# IR sensors:
ros2 topic echo /ir_front_left
ros2 topic echo /ir_front_center
ros2 topic echo /ir_front_right

# IMU:
ros2 topic echo /imu/data

# Wheel joint states:
ros2 topic echo /joint_states
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ros2` not found | `source /opt/ros/kilted/setup.bash` |
| Package not found | `source ~/slodda_1/install/setup.bash` or rebuild with `cb` |
| Topics not visible across machines | Check `ROS_DOMAIN_ID=42` on both; check same WiFi subnet |
| Nav2 not activating | Wait 30 s after `sim.launch.py`; check `ros2 node list` for lifecycle nodes |
| Gazebo crash on startup | Try `--render-engine-gui ogre` flag; check `gz sim --version` |
| Build fails after conflict | `rm -rf build/ install/ log/` then `cb` |
| Pi SD card corruption | Always `sudo shutdown now` before cutting power |

---

## Useful One-Liners

```bash
# Kill all ros2 nodes:
pkill -f ros2

# Monitor CPU on Pi:
watch -n 1 'cat /proc/loadavg'

# Find which node publishes to a topic:
ros2 topic info -v /cmd_vel

# Record a bag for replay/debugging:
ros2 bag record /cmd_vel /odom /scan /yolo/detections

# Play back a bag:
ros2 bag play <bag_folder>/

# Print transform between frames:
ros2 run tf2_ros tf2_echo map base_link
```
