# Slødda 1 — Raspberry Pi Oppsett

Denne guiden tar deg fra blank SD-kort til en fullt fungerende ROS 2 Kilted-installasjon på Raspberry Pi 4, klar til bruk i Slødda 1-prosjektet.

---

## Maskinvare

- Raspberry Pi 4 Model B
- SD-kort: 32 GB (minimum)
- Tilgang til hotspot eller WiFi (ikke eduroam — fungerer ikke med standard netplan)

---

## Steg 1: Flash Ubuntu Server til SD-kortet

1. Last ned [Raspberry Pi Imager](https://www.raspberrypi.com/software/) på Ubuntu-PC-en:
   ```bash
   sudo apt install rpi-imager
   rpi-imager
   ```

2. Velg i Imager:
   - **Device:** Raspberry Pi 4
   - **OS:** Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.4 LTS (64-bit)**
   - **Storage:** SD-kortet ditt

3. Klikk **Next → Edit Settings** og fyll inn:
   - Hostname: `SloddaPi`
   - Username: `slodda1`
   - Password: (velg selv)
   - **WiFi SSID og passord** (bruk mobil-hotspot, ikke eduroam)
   - Timezone: `Europe/Oslo`
   - ✅ Enable SSH → Password authentication

4. Lagre og flash. Ta ut SD-kortet, sett i Pi, koble til strøm.

> **Viktig:** Koble Ubuntu-PC-en til samme WiFi/hotspot som Pi-en før du prøver SSH.

---

## Steg 2: Koble til via SSH

Vent 60–90 sekunder etter oppstart, deretter:

```bash
ssh slodda1@SloddaPi.local
```

Hvis `.local` ikke fungerer, finn IP-adressen på Pi-en (via skjerm eller hotspot-innstillinger) og bruk:

```bash
ssh slodda1@<IP-ADRESSE>
```

---

## Steg 3: Oppdater systemet

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot -f
```

SSH inn igjen etter reboot.

---

## Steg 4: Installer ROS 2 Kilted

```bash
sudo apt install -y software-properties-common curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-kilted-ros-base
sudo apt install -y python3-colcon-common-extensions build-essential
```

---

## Steg 5: Sett opp miljøvariabler

```bash
echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
```

Verifiser:
```bash
ros2 topic list
# Forventet output: /parameter_events og /rosout
```

---

## Steg 6: Klon repoet

Generer et GitHub Personal Access Token på [github.com/settings/tokens](https://github.com/settings/tokens) (huk av `repo`), deretter:

```bash
git clone https://sigurdkroye-afk:<TOKEN>@github.com/sigurdkroye-afk/slodda_1.git
```

---

## Steg 7: Sett opp colcon workspace

```bash
mkdir -p ~/slodda_ws/src
ln -s ~/slodda_1/slodda_description ~/slodda_ws/src/
ln -s ~/slodda_1/slodda_gazebo ~/slodda_ws/src/
ln -s ~/slodda_1/slodda_bringup ~/slodda_ws/src/

echo "source ~/slodda_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Bygg workspace:
```bash
cd ~/slodda_ws
colcon build --symlink-install
source ~/.bashrc
```

Verifiser pakker:
```bash
ros2 pkg list | grep slodda
# Forventet: slodda_bringup, slodda_description, slodda_gazebo
```

---

## Workspace-struktur

```
~/
├── slodda_1/               # Git-repoet (kildekode)
│   ├── slodda_bringup/     # Launch-filer, Nav2-konfig, Python-noder
│   ├── slodda_description/ # URDF/Xacro robot-modell, meshes
│   ├── slodda_gazebo/      # Gazebo-verden, kart, sim-launch
│   ├── firmware/           # ESP32-kode
│   └── hardware/           # Pi-spesifikk hardware-kode
│
└── slodda_ws/              # Colcon workspace
    └── src/                # Symlinker til pakker i slodda_1/
```

---

## Nyttige kommandoer

| Kommando | Beskrivelse |
|---|---|
| `cd ~/slodda_ws && colcon build --symlink-install` | Bygg alle pakker |
| `source ~/.bashrc` | Last inn miljøvariabler |
| `ros2 topic list` | List alle aktive topics |
| `ros2 node list` | List alle kjørende noder |
| `ros2 launch slodda_bringup <launch-fil>` | Kjør en launch-fil |
| `sudo shutdown now` | Slå av Pi trygt |
| `sudo reboot -f` | Restart Pi |

---

## Viktige notater

- **Ikke bruk eduroam** — krever 802.1x enterprise-autentisering som ikke fungerer med standard netplan. Bruk mobil-hotspot eller hjemme-WiFi.
- **Alltid slå av Pi trygt** med `sudo shutdown now` — SD-kort tåler ikke plutselig strømbrudd.
- **ROS_DOMAIN_ID=42** må være satt på alle maskiner som skal kommunisere med hverandre.
- **Symlink-install** betyr at Python-filer og launch-filer ikke trenger rebuild etter endringer — bare C++-kode trenger det.
- Filer og konfigurasjon **lagres permanent** på SD-kortet mellom omstarter som en vanlig Linux-maskin.
