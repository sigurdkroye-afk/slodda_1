# Slødda 1 — Cheatsheet

Workspace: `~/slodda_1` | ROS: Kilted | Domain ID: `42` | Pi: `slodda1@sloddapi` / `100.66.136.28`

---

## Build

```bash
cd ~/slodda_1
colcon build --symlink-install                          # full bygg
colcon build --packages-select slodda_bringup          # etter config-endringer
rm -rf build/ install/ log/ && colcon build            # clean bygg
source install/setup.bash
```

## Launch — Pi

```bash
ros2 launch slodda_bringup hardware.launch.py          # full stack (~35s oppstart)
```

## Laptop — RViz

```bash
export ROS_DOMAIN_ID=42
ros2 run rviz2 rviz2 -d ~/slodda_1/src/slodda_bringup/config/hardware.rviz
```

## Navigasjon

```bash
# Rett frem 1 m
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 360° spin på stedet
ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 6.28}"

# Backup 0.3 m
ros2 action send_goal /backup nav2_msgs/action/BackUp \
  "{target: {x: 0.3, y: 0.0, z: 0.0}, speed: 0.1, time_allowance: {sec: 10}}"
```

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

Manuell kalibrering (krever arm_controller_node kjørende):
```bash
python3 ~/slodda_1/src/slodda_bringup/scripts/arm_manual_control.py
# t/g/y/h/u/j = servo-steg | z/x/c/v = lagre pos 0/1/2/3 | q = nødstopp
```

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"   # stopp
```

## Topics

```bash
ros2 topic list
ros2 topic hz /scan                        # ~10 Hz
ros2 topic hz /imu/data                    # ~50 Hz
ros2 topic hz /odom                        # ~10 Hz
ros2 topic hz /odometry/filtered           # ~12 Hz
ros2 topic echo /odometry/filtered --once
ros2 topic echo /arm/status                # arm-feedback fra ESP32
```

## TF / frames

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_tools view_frames
```

## Nodes / lifecycle

```bash
ros2 node list
ros2 node info /ekf_filter_node
ros2 node info /arm_controller
ros2 lifecycle nodes
ros2 lifecycle get /controller_server
```

## Pi I/O-verifisering

```bash
i2cdetect -y 1                             # skal vise 0x28 (BNO055)
ls /dev/ttyAMA3                            # lidar-port
ls /dev/ttyS0                              # arm UART til ESP32
cat /proc/cmdline | grep serial            # skal IKKE inneholde console=serial0
systemctl is-enabled serial-getty@ttyS0    # skal vise disabled
cat /boot/firmware/config.txt | grep -E "uart3|i2c_arm|enable_uart"
```

## SSH / deploy

```bash
ssh slodda1@sloddapi
cd ~/slodda_1 && git pull && colcon build --packages-select slodda_bringup
source install/setup.bash
```

## Diagnostikk

```bash
htop
ros2 topic echo /diagnostics --once
pkill -f ros2                              # drep alle ROS2-prosesser
```

## Bag-recording

```bash
ros2 bag record /scan /odom /odometry/filtered /imu/data /cmd_vel /arm/status /tf /tf_static
ros2 bag play <bag_dir>/
```

## Git

```bash
git checkout dev && git pull
git checkout main && git merge dev && git push
```

---

## Kjente gotchas

| Problem | Fiks |
|---|---|
| Servoer gjør random shit ved oppstart | `console=serial0` i cmdline.txt — kjør: `sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt` + reboot |
| `/dev/ttyS0: Permission denied` | `sudo systemctl disable --now serial-getty@ttyS0` + `sudo chmod 666 /dev/ttyS0` |
| Arm TIMEOUT | Feil pins koblet til ESP32 — verifiser GPIO13 (RX) og GPIO17 (TX) |
| Launch-endringer virker ikke etter pull | `colcon build --packages-select slodda_bringup` |
| Topics ikke synlig fra laptop | `export ROS_DOMAIN_ID=42` på begge maskiner |
| BNO055 ikke funnet | `i2cdetect -y 1` → sjekk `0x28`; verifiser `dtparam=i2c_arm=on` i config.txt |
| Spin gjør for mange runder | Bekreft `odom0_config` vyaw=false i `ekf.yaml` |
| Nav2 starter ikke | Vent 60 s; `htop`; reboot Pi om CPU >90% |
