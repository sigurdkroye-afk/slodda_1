# Slødda 1 — Tester

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

Trykk **2D Nav Goal** i RViz → klikk punkt → robot kjører dit.

---

## TEST 2 — Keyboard + arm search + IR-grab

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

## Nødstopp

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```
