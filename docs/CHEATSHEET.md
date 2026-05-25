# Slødda 1 — Cheatsheet

## Setup (begge maskiner)

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/kilted/setup.bash
source ~/slodda_1/install/setup.bash
```

---

## TEST 1 — Nav2 alene

**Pi:**
```bash
cd ~/slodda_1 && git pull && source install/setup.bash
ros2 launch slodda_bringup hardware.launch.py
```

**Laptop:**
```bash
cd ~/slodda_1 && source install/setup.bash
ros2 launch slodda_bringup laptop.launch.py
```

---

## TEST 2 — Keyboard + arm + IR-grab

**Pi Terminal 1:**
```bash
cd ~/slodda_1 && git pull && source install/setup.bash
ros2 launch slodda_bringup hardware.launch.py
```

**Pi Terminal 2:**
```bash
source ~/slodda_1/install/setup.bash
ros2 service call /arm/search std_srvs/srv/Trigger {}
```

**Pi Terminal 3:**
```bash
source ~/slodda_1/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Arm

```
0=OPEN  1=DRIVE  2=SEARCH (IR aktiv)  3=GRAB
```

```bash
ros2 service call /arm/open   std_srvs/srv/Trigger {}
ros2 service call /arm/drive  std_srvs/srv/Trigger {}
ros2 service call /arm/search std_srvs/srv/Trigger {}
ros2 service call /arm/grab   std_srvs/srv/Trigger {}
ros2 service call /arm/stop   std_srvs/srv/Trigger {}

ros2 topic echo /arm/status
```

Manuell kalibrering:
```bash
python3 ~/slodda_1/src/slodda_bringup/scripts/arm_manual_control.py
```

---

## Bear Mission

```bash
ros2 topic pub --once /bear_mission/start std_msgs/msg/Bool "data: true"
ros2 topic echo /bear_mission/status
bash ~/slodda_1/src/slodda_bringup/scripts/monitor_mission.sh
```

Med parametere:
```bash
ros2 launch slodda_bringup mission_full.launch.py goal_x:=1.0 goal_y:=0.0 grab_timeout_s:=30
```

---

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Topics

```bash
ros2 topic hz /odometry/filtered
ros2 topic hz /imu/data
ros2 topic hz /scan
ros2 topic hz /cmd_vel
ros2 topic hz /camera/image_raw
```

---

## TF / Lifecycle

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 node list | grep -E 'nav2|ekf|mission'
```

---

## RViz

```bash
ros2 run rviz2 rviz2 -d ~/slodda_1/src/slodda_bringup/config/hardware.rviz
```

---

## Bag

```bash
ros2 bag record /odometry/filtered /imu/data /scan /tf /tf_static /cmd_vel /arm/status
ros2 bag play <bag-dir>
ros2 bag info <bag-dir>
```

---

## Build / Deploy

```bash
# Laptop:
colcon build --symlink-install && source install/setup.bash
git push

# Pi:
cd ~/slodda_1 && git pull
bash src/slodda_bringup/scripts/install.sh
```

---

## Debug

```bash
htop
i2cdetect -y 1
ls -l /dev/ttyS0 /dev/ttyAMA3
bash ~/slodda_1/src/slodda_bringup/scripts/clean_dds.sh
```

---

## Nødstopp

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```
