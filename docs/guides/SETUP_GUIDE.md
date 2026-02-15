# Slødda 1 - Setup Guide for Gruppemedlemmer

Dette er en komplett guide for å sette opp utviklingsmiljøet ditt for Slødda 1 prosjektet.

## 📋 Forutsetninger

- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Kilted
- **Git** installert
- **GitHub** konto med tilgang til repo

---

## 🚀 Del 1: Git & GitHub Setup (15 min)

### 1.1 Clone repository
```bash
cd ~
git clone https://github.com/sigurdkroye-afk/slodda_1.git
cd slodda_1
```

### 1.2 Verifiser innhold
```bash
ls
```

Du skal se:
- docs/
- firmware/
- scripts/
- slodda_bringup/
- README.md

### 1.3 Konfigurer Git (første gang)
```bash
git config --global user.name "Ditt Navn"
git config --global user.email "din.epost@ntnu.no"
```

### 1.4 Autentiser med GitHub
```bash
sudo apt install gh -y
gh auth login
```

Følg instruksjonene:
- Velg: GitHub.com
- Protocol: HTTPS
- Authenticate: Yes
- Method: Login with web browser
- Kopier koden og åpne browser

---

## 🤖 Del 2: ROS 2 Workspace Setup (20 min)

### 2.1 Verifiser ROS 2 installasjon
```bash
echo $ROS_DISTRO
```

Skal vise: `kilted`

Hvis tom - ROS 2 er ikke installert! (Spør gruppa om hjelp)

### 2.2 Installer nødvendige pakker
```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-kilted-joint-state-publisher \
  ros-kilted-joint-state-publisher-gui \
  ros-kilted-robot-state-publisher \
  ros-kilted-xacro \
  ros-kilted-teleop-twist-keyboard
```

### 2.3 Initialiser rosdep
```bash
sudo rosdep init
rosdep update
```

(Hvis du får "already initialized" - hopp over første kommando)

### 2.4 Opprett workspace
```bash
mkdir -p ~/slodda_ws/src
cd ~/slodda_ws/src
ln -s ~/slodda_1 .
```

### 2.5 Bygg workspace
```bash
cd ~/slodda_ws
colcon build --symlink-install
```

### 2.6 Lag setup script
```bash
cd ~
cat > setup_slodda.sh << 'EOF'
#!/bin/bash

# ROS 2 Kilted
source /opt/ros/kilted/setup.bash

# Workspace
if [ -f ~/slodda_ws/install/setup.bash ]; then
    source ~/slodda_ws/install/setup.bash
fi

# ROS 2 Settings
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

# Aliases
alias cb='cd ~/slodda_ws && colcon build --symlink-install'
alias cs='source ~/slodda_ws/install/setup.bash'
alias cw='cd ~/slodda_ws'

echo "🤖 Slødda 1 environment loaded!"
echo "   Workspace: ~/slodda_ws"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""
echo "Aliases:"
echo "  cb - colcon build"
echo "  cs - source workspace"
echo "  cw - cd to workspace"
EOF

chmod +x setup_slodda.sh
```

### 2.7 Legg til i .bashrc
```bash
echo "" >> ~/.bashrc
echo "# Slødda 1 Project" >> ~/.bashrc
echo "source ~/setup_slodda.sh" >> ~/.bashrc
```

### 2.8 Test setup

**Åpne ny terminal (Ctrl+Alt+T)**

Du skal automatisk se:
```
🤖 Slødda 1 environment loaded!
```

---

## ✅ Del 3: Test at alt fungerer (5 min)

### 3.1 Test hello_robot node
```bash
ros2 run slodda_bringup hello_robot
```

Du skal se:
```
[INFO] [hello_robot]: 🤖 Hello Robot node started!
[INFO] [hello_robot]: Publishing cmd_vel [count=0]: linear.x=0.20
...
```

**La den kjøre!**

### 3.2 Åpne ny terminal og verifiser
```bash
ros2 topic list
```

Du skal se `/cmd_vel` i lista.
```bash
ros2 topic echo /cmd_vel
```

Du skal se Twist-meldinger!

**Stopp begge terminaler med Ctrl+C**

---

## 🎯 Del 4: Git Workflow (viktig!)

### Før du begynner å jobbe:
```bash
cd ~/slodda_1
git checkout dev
git pull origin dev
```

### Lag feature branch:
```bash
git checkout -b feature/min-feature
```

### Gjør endringer, test, commit:
```bash
git add .
git commit -m "feat: beskrivelse av endring"
git push origin feature/min-feature
```

### Lag Pull Request på GitHub

1. Gå til repo på GitHub
2. Klikk "Pull requests"
3. Klikk "New pull request"
4. Base: `dev`, Compare: `feature/min-feature`
5. Beskriv endringene
6. Be om review fra gruppemedlem
7. Merge når godkjent

---

## 🔧 Nyttige kommandoer

### Bygg workspace:
```bash
cb  # (alias) eller: cd ~/slodda_ws && colcon build --symlink-install
```

### Source workspace:
```bash
cs  # (alias) eller: source ~/slodda_ws/install/setup.bash
```

### Gå til workspace:
```bash
cw  # (alias) eller: cd ~/slodda_ws
```

### Bygg én pakke:
```bash
colcon build --packages-select pakkenavn --symlink-install
```

### List ROS topics:
```bash
ros2 topic list
```

### Echo en topic:
```bash
ros2 topic echo /topic_navn
```

### List ROS nodes:
```bash
ros2 node list
```

---

## 🐛 Troubleshooting

### Problem: `ros2` kommandoer fungerer ikke

**Løsning:**
```bash
source ~/setup_slodda.sh
echo $ROS_DISTRO  # skal vise "kilted"
```

### Problem: Kan ikke se topics fra andre terminaler

**Løsning:** Sjekk at begge terminaler har samme ROS_DOMAIN_ID:
```bash
echo $ROS_DOMAIN_ID  # skal vise "42" i alle terminaler
```

### Problem: `colcon build` feiler

**Løsning:**
```bash
cd ~/slodda_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Problem: Git push feiler med authentication error

**Løsning:**
```bash
gh auth login
# Følg instruksjonene
```

---

## 📚 Ressurser

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Git Basics](https://git-scm.com/book/en/v2)
- Spør i gruppechatten!

---

## ✅ Checklist - Er du klar?

- [ ] Git clone fungerer
- [ ] ROS 2 miljø lastes automatisk i ny terminal
- [ ] `ros2 run slodda_bringup hello_robot` fungerer
- [ ] Kan se topics med `ros2 topic list`
- [ ] Kan bygge workspace med `cb` alias
- [ ] Kan pushe til GitHub
- [ ] Forstår Git workflow (feature branches → PR → dev)

**Hvis alt over er ✅ - du er klar til å jobbe! 🎉**

---

**Spørsmål? Kontakt:**
- Sigurd Kristian Øye
- [Gruppemedlem 2]
- [Gruppemedlem 3]
