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

## Branches

| Branch | Formål |
|--------|--------|
| `main` | Stabil, klar for demo |
| `dev`  | Integrasjonsgren – alle features merges hit |
| `feature/<navn>` | Ny funksjonalitet |
| `fix/<navn>`     | Feilretting |

## Workflow – Pull Request og Merge

### 1. Opprett en ny branch fra `dev`
```bash
git checkout dev
git pull origin dev
git checkout -b feature/min-feature
```

### 2. Gjør endringer og commit
```bash
git add .
git commit -m "feat: kort beskrivelse av endringen"
```

### 3. Push branchen til GitHub
```bash
git push origin feature/min-feature
```

### 4. Lag Pull Request (PR)
- Gå til repositoryet på GitHub
- Klikk **"Compare & pull request"**
- Sett **base** til `dev` og **compare** til `feature/min-feature`
- Gi PR-en en beskrivende tittel og fyll ut beskrivelsen
- Be om review fra et teammedlem
- Klikk **"Create pull request"**

### 5. Code review og godkjenning
- Revieweren kommenterer eventuelt på koden
- Gjør nødvendige endringer og push til samme branch – PR oppdateres automatisk
- Reviewer godkjenner PR-en (**"Approve"**)

### 6. Merge PR til `dev`
- Bruk **"Squash and merge"** for feature-branches (ryddig historikk)
- Slett branchen etter merge (**"Delete branch"**)

### 7. Merge `dev` til `main` (ved demo/release)
```bash
git checkout main
git pull origin main
git merge --no-ff dev -m "release: v<versjon>"
git push origin main
```

### Eksempel på commit-meldinger
```
feat: legg til arm-kinematikk
fix: rett opp kollisjonsdeteksjon i Gazebo
docs: oppdater installasjonsveiledning
refactor: rydd opp i launch-filer
```

## Team

- [Jostein aka. SLAKTERN]
- [Isak aka. GRØTEN]
- [Sigurd aka. Tyngden]
