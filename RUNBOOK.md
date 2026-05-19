# PHOENIX SAR — Command Runbook

All confirmed-working commands. Copy-paste ready.

**Pi IP**: `172.21.164.200`  **Pi user**: `phoenix`  
**Ports**: ESP32 motor = `/dev/ttyUSB0` · RPLIDAR = `/dev/ttyUSB1` · XIAO FTM = `/dev/ttyACM0`

---

## Before Every Session

```bash
# SSH in
ssh phoenix@172.21.164.200

# Verify ports (they can swap after replug)
ls /dev/ttyUSB* /dev/ttyACM*

# If previous session left zombie processes:
killros
```

If robot isn't responding (ReadByte timeouts): **physically unplug and replug the ESP32 USB cable** (`/dev/ttyUSB0`), wait 3 seconds, relaunch.

---

## Easiest Way — Web Launcher

Start this once. Then control everything from your browser at `http://172.21.164.200:8081`.

```bash
# [PI]
source ~/ros2_ws/install/setup.bash
ros2 run articubot_one phoenix_launcher
```

---

## MODE 1 — Manual Control

**Terminal 1 [PI]:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch articubot_one slam_nav_launch.py serial_port:=/dev/ttyUSB1
```

Wait for: `Managed nodes are active`

**Terminal 2 [PI]:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_joy -p stamped:=true -p frame_id:=base_link
```

Keys: `i` forward · `,` backward · `j` left · `l` right · `k` stop  
Speed: `q`/`z` increase/decrease

---

## MODE 2 — Autonomous SAR Mission

**Terminal 1 [PI]:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch articubot_one maze_mission_launch.py \
  serial_port:=/dev/ttyUSB1 \
  model_path:=/home/phoenix/model/target_classifier_int8.tflite \
  metadata_path:=/home/phoenix/model/metadata.json
```

Dashboard opens automatically at: **`http://172.21.164.200:8080`**

What happens:
1. Robot explores the maze autonomously using frontier exploration
2. Spins 360° at each frontier to scan for target with camera
3. Stops and saves map when target detected (prob > 70%, 3/5 votes)
4. Computes shortest path start→target and publishes to `/phoenix/shortest_path`
5. Maps auto-save every 30s to `/home/phoenix/ros2_ws/maps/maze_mission.*`

---

## MODE 3 — RTT Data Collection

**Terminal 1 [PI] — SLAM:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch articubot_one slam_nav_launch.py serial_port:=/dev/ttyUSB1
```

**Terminal 2 [PI] — Teleop:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_joy -p stamped:=true -p frame_id:=base_link
```

**Terminal 3 [PI] — Logger:**
```bash
source ~/ros2_ws/install/setup.bash
RUN=run_$(date +%Y%m%d_%H%M%S)
mkdir -p /home/phoenix/datasets/$RUN
ros2 run articubot_one phoenix_logger \
  --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baud:=115200 \
  -p output_dir:=/home/phoenix/datasets/$RUN
```

Logger is working when you see: `rtt.csv=...B pending_bursts=0 ... map=Y`

**Send labels while driving [PI] Terminal 4:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /phoenix/label std_msgs/msg/String "{data: 'LOS_DYNAMIC'}"
ros2 topic pub --once /phoenix/label std_msgs/msg/String "{data: 'NLOS_WALL'}"
ros2 topic pub --once /phoenix/label std_msgs/msg/String "{data: 'NLOS_DOOR'}"
ros2 topic pub --once /phoenix/label std_msgs/msg/String "{data: 'LOS_STATIC'}"
```

Available labels: `LOS_STATIC` · `LOS_DYNAMIC` · `NLOS_WALL` · `NLOS_CORNER` · `NLOS_DOOR` · `NLOS_DYNAMIC`

**Verify data after stopping logger:**
```bash
ls -la /home/phoenix/datasets/$RUN/
python3 -c "
import csv
rows = list(csv.DictReader(open('/home/phoenix/datasets/$RUN/rtt.csv')))
dists = [float(r['d_median_m']) for r in rows if r['d_median_m']]
print(f'Bursts: {len(dists)}, Min: {min(dists):.2f}m, Max: {max(dists):.2f}m')
"
```

---

## MODE 4 — IMPPF (runs on your laptop, not Pi)

**Transfer data from Pi [LAPTOP PowerShell]:**
```powershell
scp -r phoenix@172.21.164.200:/home/phoenix/datasets/run01 "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01"
```

**Run IMPPF [LAPTOP PowerShell]:**
```powershell
cd "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\articubot_one\phoenixIMPPF\phoenixIMPPF"

python 04_imppf_prototype.py `
  --map      "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01\map.npy" `
  --map-meta "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01\map_meta.json" `
  --traj     "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01\trajectory.csv" `
  --rtt      "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01\rtt.csv" `
  --particles 1000 `
  --sigma-los 0.5 --sigma-nlos 1.5 --bias 0.8 `
  --offset-b 5.5 `
  --likelihood student_t `
  --rough 0.3 `
  --save "C:\Users\omarj\OneDrive\Desktop\FINAL GRAD\run01\result"
```

Key parameters:
| Param | Meaning | Default |
|-------|---------|---------|
| `--offset-b` | ESP32-S3 hardware range offset (m). Subtract from every measurement. Calibrate: place 1m from AP, measure mean raw distance, offset-b = mean - 1.0 | 0.0 |
| `--sigma-los` | LOS noise std dev (m) after removing offset | 0.5 |
| `--sigma-nlos` | NLOS noise std dev (m) | 1.5 |
| `--bias` | Extra distance added by thin walls (m) | 0.8 |
| `--rough` | Post-resample jitter to prevent particle collapse | 0.10 |
| `--likelihood` | `gaussian` (fast) or `student_t` (outlier-robust, recommended) | gaussian |

---

## Utility Commands

**Kill all ROS processes:**
```bash
killros
```

**Verify ESP32 firmware is responding:**
```bash
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
s.write(b'\r'); time.sleep(0.2); print('ping:', repr(s.read(20)))
s.write(b'e\r'); time.sleep(0.2); print('encoders:', repr(s.read(20)))
s.close()
"
# Expected: ping: b'I ack\r\n'  encoders: b'e 0 0\r\n'
```

**Verify XIAO FTM firmware is running:**
```bash
python3 -c "
import serial
s = serial.Serial('/dev/ttyACM0', 115200, timeout=3)
for _ in range(10):
    line = s.readline()
    if line: print(line.decode('utf-8', errors='ignore').strip())
s.close()
"
# Expected: BURST_TX,...  BURST_START,...  FTM_F,...
```

**Check what's publishing on a topic:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /scan           # LIDAR (~10 Hz)
ros2 topic hz /odom           # Odometry (~30 Hz when moving)
ros2 topic hz /map            # SLAM map (updates on change)
ros2 topic echo /phoenix/status --once   # Mission status
```

**Rebuild after code changes:**
```bash
cd ~/ros2_ws
colcon build --packages-select articubot_one --symlink-install 2>&1 | tail -3
source ~/ros2_ws/install/setup.bash
```

**Copy installed nav2 params when source changes don't take effect:**
```bash
cp ~/ros2_ws/src/articubot_one/config/nav2_params.yaml \
   ~/ros2_ws/install/articubot_one/share/articubot_one/config/nav2_params.yaml
```

---

## Port Assignment Reference

```
/dev/ttyUSB0  →  Motor controller ESP32     (firmware: esp32_diff_drive.ino)
/dev/ttyUSB1  →  RPLIDAR A1                 (launch arg: serial_port:=/dev/ttyUSB1)
/dev/ttyACM0  →  XIAO ESP32-S3 FTM sensor   (logger arg: serial_port:=/dev/ttyACM0)
```

If ports swap, identify them:
```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct" | head -4
udevadm info -a -n /dev/ttyUSB1 | grep -E "idVendor|idProduct" | head -4
```

---

## Key File Locations on Pi

```
~/ros2_ws/src/articubot_one/
  config/nav2_params.yaml              ← Nav2 tuning (speed, inflation, etc.)
  config/my_controllers.yaml           ← Wheel geometry
  description/ros2_control.xacro      ← ESP32 port + encoder CPR
  scripts/phoenix_explorer.py         ← Frontier exploration logic
  scripts/camera_detector_node.py     ← TFLite target detection
  scripts/phoenix_dashboard.py        ← Web dashboard (:8080)
  scripts/phoenix_launcher.py         ← Web launcher (:8081)
  phoenixIMPPF/phoenixIMPPF/new.c     ← XIAO ESP32-S3 firmware
  phoenixIMPPF/phoenixIMPPF/03_phoenix_logger.py  ← RTT data logger (installed as phoenix_logger)
  phoenixIMPPF/phoenixIMPPF/04_imppf_prototype.py ← IMPPF particle filter (run on laptop)

~/ros2_ws/maps/                        ← Saved maps (.pgm, .yaml, .npy)
~/ros2_ws/maps/camera_runs/            ← Camera video recordings
~/model/                               ← TFLite model files
~/datasets/                            ← RTT/FTM datasets
```
