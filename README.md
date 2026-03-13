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

- [Jostein aka. SLAKTERN]
- [Isak aka. GRØTEN]
- [Sigurd aka. Tyngden]
