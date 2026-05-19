# PHOENIX SAR — Autonomous Search & Rescue Robot

Raspberry Pi 5 · RPLIDAR A1 · ESP32 differential drive · Pi Camera Module 3 · XIAO ESP32-S3 (FTM ranging)

---

## What it does

- **Manual control** — drive via keyboard or browser virtual gamepad
- **Autonomous SAR mission** — frontier exploration with live SLAM, stops when camera detects target
- **RTT data collection** — collect Wi-Fi FTM ranging data synchronized to SLAM poses for IMPPF localization
- **Web dashboard** — live SLAM map, camera feed, mission status at `http://PI_IP:8080`
- **Web launcher** — start/stop all modes from the browser at `http://PI_IP:8081` (no terminal needed)

---

## Hardware

| Device | Port |
|--------|------|
| Motor controller ESP32 | `/dev/ttyUSB0` |
| RPLIDAR A1 | `/dev/ttyUSB1` |
| XIAO ESP32-S3 (FTM) | `/dev/ttyACM0` |
| Pi Camera Module 3 | rpicam-vid (built-in) |

> Ports can swap after replug. Always verify with `ls /dev/ttyUSB* /dev/ttyACM*`.

---

## Quick Start (Pi already set up)

```bash
# SSH into Pi
ssh phoenix@172.21.164.200

# Start the web launcher (no further terminal commands needed)
source ~/ros2_ws/install/setup.bash
ros2 run articubot_one phoenix_launcher
```

Then open **`http://172.21.164.200:8081`** in your browser and click a mode button.

---

## First-Time Pi Setup

### 1. Clone the repo into the ROS 2 workspace

```bash
ssh phoenix@172.21.164.200
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/phoenix-sar.git articubot_one
```

### 2. Install dependencies

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws

# ROS 2 package dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Extra packages
sudo apt install -y ros-jazzy-nav2-regulated-pure-pursuit-controller python3-serial

# Source dependencies (serial + diffdrive_arduino must be in src/)
cd ~/ros2_ws/src
git clone https://github.com/joshnewans/serial.git
git clone https://github.com/joshnewans/diffdrive_arduino.git
```

### 3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install 2>&1 | tail -5

# Add to shell (run once)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Serial port permissions

```bash
sudo usermod -a -G dialout phoenix
# Log out and back in, OR:
newgrp dialout
```

### 5. Flash firmware

**Motor ESP32** — Arduino IDE, laptop only:
```
firmware/esp32_diff_drive/esp32_diff_drive.ino
Board: ESP32 Dev Module  |  Baud: 57600
```

**XIAO ESP32-S3 (FTM)** — PlatformIO, laptop only:
```
phoenixIMPPF/phoenixIMPPF/new.c  →  src/main.c
platformio.ini: framework = espidf
sdkconfig.defaults: CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
```

### 6. TFLite model for target detection

```bash
mkdir -p /home/phoenix/model
# Copy your model files to Pi:
# /home/phoenix/model/target_classifier_int8.tflite
# /home/phoenix/model/metadata.json
```

### 7. killros alias (quality of life)

```bash
echo "alias killros='pkill -f ros2_control_node; pkill -f rplidar_composition; pkill -f async_slam_toolbox; pkill -f controller_server; pkill -f phoenix_dashboard; pkill -f camera_detector; pkill -f phoenix_explorer; sudo rm -f /var/lock/LCK..ttyUSB*; sleep 1'" >> ~/.bashrc
source ~/.bashrc
```

---

## Web Interfaces

| URL | What it shows |
|-----|--------------|
| `http://172.21.164.200:8081` | **Launcher** — start/stop modes, virtual gamepad |
| `http://172.21.164.200:8080` | **Dashboard** — SLAM map, camera feed, mission status (starts with Autonomous mode) |

---

## IMPPF (AP Localisation) — Laptop Side

After collecting an RTT dataset on the Pi, transfer it and run the particle filter:

```powershell
# Transfer data from Pi (PowerShell)
scp -r phoenix@172.21.164.200:/home/phoenix/datasets/run01 "C:\path\to\run01"

# Install Python dependencies (once)
pip install numpy matplotlib

# Run IMPPF
cd articubot_one\phoenixIMPPF\phoenixIMPPF
python 04_imppf_prototype.py `
  --map      "C:\path\to\run01\map.npy" `
  --map-meta "C:\path\to\run01\map_meta.json" `
  --traj     "C:\path\to\run01\trajectory.csv" `
  --rtt      "C:\path\to\run01\rtt.csv" `
  --offset-b 5.5 --sigma-los 0.5 --sigma-nlos 1.5 --bias 0.8 `
  --likelihood student_t --rough 0.3 `
  --save "C:\path\to\run01\result"
```

See `RUNBOOK.md` for all parameters and calibration notes.

---

## Pushing updates to GitHub

```bash
# On Pi — pull latest changes
cd ~/ros2_ws/src/articubot_one
git pull

# Rebuild after pull
cd ~/ros2_ws
colcon build --symlink-install --packages-select articubot_one 2>&1 | tail -3
source ~/ros2_ws/install/setup.bash
```

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `ReadByte() call has timed out` | Power cycle ESP32 (unplug/replug `/dev/ttyUSB0`) |
| Serial port busy | Run `killros` then wait 5s |
| Ports swapped | `ls /dev/ttyUSB*` — unplug one device at a time to identify |
| Robot won't move with teleop | Must use `stamped:=true` — see RUNBOOK |
| Dashboard map not loading | Wait 30s for SLAM to initialise |
| FTM `Failed to allocate` | Firmware config issue — `FTM_FRMS_PER_BURST` must be 32 with `use_get_report_api=true` |
