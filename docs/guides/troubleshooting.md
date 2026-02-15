# Troubleshooting Guide - Slødda 1

Vanlige problemer og løsninger.

---

## ROS 2 Problemer

### `ros2` kommandoer ikke funnet

**Symptom:**
```
bash: ros2: command not found
```

**Løsning:**
```bash
source ~/setup_slodda.sh
```

Sjekk:
```bash
echo $ROS_DISTRO  # skal vise "kilted"
```

---

### Kan ikke se topics mellom terminaler

**Symptom:**
- Terminal 1 publiserer til `/cmd_vel`
- Terminal 2 ser ikke `/cmd_vel` med `ros2 topic list`

**Løsning:**
```bash
# I begge terminaler:
echo $ROS_DOMAIN_ID
```

Hvis forskjellige:
```bash
source ~/setup_slodda.sh
```

---

### RMW Implementation error

**Symptom:**
```
Error getting RMW implementation identifier
librmw_cyclonedds_cpp.so: cannot open shared object file
```

**Løsning:**

Rediger setup script:
```bash
nano ~/setup_slodda.sh
```

Endre:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Source på nytt:
```bash
source ~/setup_slodda.sh
```

---

## Git Problemer

### Authentication failed

**Symptom:**
```
fatal: Authentication failed
```

**Løsning:**
```bash
gh auth login
```

Følg instruksjonene (velg web browser).

---

### Permission denied (publickey)

**Løsning:**

Bruk HTTPS istedenfor SSH:
```bash
git remote set-url origin https://github.com/sigurdkroye-afk/slodda_1.git
```

---

### Merge conflict

**Løsning:**

1. Pull siste endringer:
```bash
git pull origin dev
```

2. Åpne konflikt-filer og rediger manuelt
3. Fjern `<<<<<<<`, `=======`, `>>>>>>>` markørene
4. Velg hvilken kode som skal beholdes
5. Stage og commit:
```bash
git add .
git commit -m "fix: resolve merge conflict"
```

---

## Colcon Build Problemer

### Build feiler

**Symptom:**
```
--- stderr: pakkenavn
[diverse feilmeldinger]
```

**Løsning 1 - Rens og bygg på nytt:**
```bash
cd ~/slodda_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

**Løsning 2 - Bygg kun én pakke:**
```bash
colcon build --packages-select pakkenavn --symlink-install
```

**Løsning 3 - Sjekk Python syntax:**
```bash
python3 -m py_compile src/pakke/pakke/node.py
```

---

### setup.py feil

**Symptom:**
```
error in setup.py
```

**Løsning:**

Sjekk at `setup.py` har:
- `import os` og `from glob import glob` øverst
- Korrekt `entry_points` for dine nodes
- Korrekt `data_files` for launch/config filer

Se `slodda_bringup/setup.py` som eksempel.

---

## Workspace Problemer

### Package ikke funnet

**Symptom:**
```bash
ros2 run pakke node
# Package 'pakke' not found
```

**Løsning:**
```bash
cd ~/slodda_ws
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep pakke
```

---

### Symlink fungerer ikke

**Symptom:**
- Endrer kode
- Ingen effekt når du kjører node

**Løsning:**

Bygg uten `--symlink-install` først:
```bash
cd ~/slodda_ws
rm -rf build/ install/
colcon build
source install/setup.bash
```

Deretter bygg med symlink:
```bash
colcon build --symlink-install
```

---

## Når ingenting fungerer

### Full reset
```bash
# Backup koden din først!
cd ~
mv slodda_ws slodda_ws.backup
mv slodda_1 slodda_1.backup

# Clone på nytt:
git clone https://github.com/sigurdkroye-afk/slodda_1.git

# Opprett workspace:
mkdir -p ~/slodda_ws/src
cd ~/slodda_ws/src
ln -s ~/slodda_1 .

# Bygg:
cd ~/slodda_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Fortsatt problemer?

1. Sjekk at du har kjørt `source ~/setup_slodda.sh`
2. Åpne ny terminal og prøv igjen
3. Spør i gruppechatten
4. Sjekk GitHub Issues for lignende problemer
5. Kontakt gruppemedlemmer

---

**Hvis du finner en ny løsning - legg den til her! 📝**
