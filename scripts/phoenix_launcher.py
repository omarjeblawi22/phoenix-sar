#!/usr/bin/env python3
"""
phoenix_launcher.py — PHOENIX SAR Web Control Launcher
=======================================================

Serves a control panel at http://<pi-ip>:8081

No terminal required after starting this node.
Open the URL in any browser on the same Wi-Fi.

Features:
  - One-click buttons for Manual, Autonomous, and RTT Collection modes
  - Virtual WASD gamepad for driving (works on phone/tablet too)
  - Live log output streamed to browser
  - Stop All button kills everything cleanly

Start:
  source ~/ros2_ws/install/setup.bash
  ros2 run articubot_one phoenix_launcher
"""

import http.server
import json
import queue
import socketserver
import subprocess
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# ── ROS 2 environment source string ──────────────────────────────────────────
_ROS = ('source /opt/ros/jazzy/setup.bash && '
        'source /home/phoenix/ros2_ws/install/setup.bash')

# ── Commands per process name ─────────────────────────────────────────────────
_CMDS = {
    'slam': (f'{_ROS} && ros2 launch articubot_one slam_nav_launch.py '
             'serial_port:=/dev/ttyUSB1'),

    'mission': (f'{_ROS} && ros2 launch articubot_one maze_mission_launch.py '
                'serial_port:=/dev/ttyUSB1 '
                'model_path:=/home/phoenix/model/target_classifier_int8.tflite '
                'metadata_path:=/home/phoenix/model/metadata.json'),
}

_MAX_LOG = 300   # lines kept in memory

# ── HTML ──────────────────────────────────────────────────────────────────────
_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PHOENIX Launcher</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;min-height:100vh;display:flex;flex-direction:column}
header{background:#161b22;border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;gap:14px}
header h1{color:#58a6ff;font-size:1.1rem;letter-spacing:3px}
.badge{padding:3px 10px;border-radius:10px;font-size:.7rem;font-weight:bold;letter-spacing:1px}
.bs{background:#21262d;color:#8b949e;border:1px solid #30363d}
.bm{background:#0d419d;color:#79c0ff;border:1px solid #1f6feb}
.ba{background:#033a16;color:#3fb950;border:1px solid #238636}
.br{background:#4d2d00;color:#e3b341;border:1px solid #9e6a03}
main{flex:1;display:grid;grid-template-columns:340px 1fr;gap:1px;background:#30363d;min-height:0}
.left{background:#0d1117;display:flex;flex-direction:column;gap:1px;overflow-y:auto}
.right{background:#0d1117;display:flex;flex-direction:column}
.section{background:#0d1117;padding:14px}
.section-title{font-size:.65rem;color:#58a6ff;letter-spacing:2px;font-weight:bold;margin-bottom:10px}
.mode-btn{width:100%;padding:14px;margin-bottom:8px;border-radius:7px;border:1px solid #30363d;
  background:#161b22;color:#c9d1d9;font-family:inherit;font-size:.85rem;cursor:pointer;
  text-align:left;transition:all .15s}
.mode-btn:hover{border-color:#58a6ff;background:#1c2333}
.mode-btn.active{border-color:#238636;background:#0d2818}
.mode-btn .mname{font-weight:bold;font-size:.95rem;display:block;margin-bottom:3px}
.mode-btn .mdesc{color:#8b949e;font-size:.72rem}
.stop-btn{width:100%;padding:12px;border-radius:7px;border:1px solid #da3633;
  background:#21262d;color:#f85149;font-family:inherit;font-size:.9rem;
  font-weight:bold;cursor:pointer;letter-spacing:1px;transition:all .15s}
.stop-btn:hover{background:#4d1010;border-color:#f85149}
/* Gamepad */
#gamepad{display:none;padding:14px;background:#0d1117;border-top:1px solid #30363d}
.gp-title{font-size:.65rem;color:#e3b341;letter-spacing:2px;font-weight:bold;margin-bottom:10px}
.gp-grid{display:grid;grid-template-columns:repeat(3,60px);grid-template-rows:repeat(3,60px);gap:6px;justify-content:center}
.gp-btn{background:#21262d;border:1px solid #30363d;border-radius:8px;color:#c9d1d9;
  font-size:1.2rem;cursor:pointer;user-select:none;display:flex;align-items:center;justify-content:center;
  transition:all .1s;-webkit-tap-highlight-color:transparent}
.gp-btn:active,.gp-btn.pressed{background:#0d419d;border-color:#58a6ff;transform:scale(.93)}
.gp-hint{text-align:center;font-size:.65rem;color:#484f58;margin-top:8px}
.speed-row{display:flex;align-items:center;gap:8px;margin-top:10px;font-size:.75rem;color:#8b949e}
.speed-row input{flex:1;accent-color:#58a6ff}
/* Log */
.log-header{background:#161b22;border-bottom:1px solid #30363d;padding:6px 12px;
  font-size:.65rem;color:#58a6ff;letter-spacing:2px;font-weight:bold;flex-shrink:0}
#log{flex:1;overflow-y:auto;padding:10px 12px;font-size:.72rem;line-height:1.6;color:#8b949e}
#log .err{color:#f85149}
#log .ok{color:#3fb950}
#log .info{color:#79c0ff}
</style>
</head>
<body>
<header>
  <h1>PHOENIX SAR</h1>
  <span id="badge" class="badge bs">STOPPED</span>
  <span id="mode-label" style="font-size:.8rem;color:#8b949e;margin-left:4px"></span>
</header>
<main>
  <div class="left">
    <div class="section">
      <div class="section-title">SELECT MODE</div>

      <button class="mode-btn" id="btn-manual" onclick="startMode('manual')">
        <span class="mname">🎮  MANUAL CONTROL</span>
        <span class="mdesc">Drive with keyboard (WASD) or on-screen buttons.
Starts SLAM so the map builds as you drive.</span>
      </button>

      <button class="mode-btn" id="btn-autonomous" onclick="startMode('autonomous')">
        <span class="mname">🤖  AUTONOMOUS MISSION</span>
        <span class="mdesc">Full SAR: frontier exploration, camera detection,
auto-stops when target found. Dashboard → :8080</span>
      </button>

      <button class="mode-btn" id="btn-rtt" onclick="startMode('rtt')">
        <span class="mname">📡  RTT DATA COLLECTION</span>
        <span class="mdesc">Starts SLAM + FTM logger. Drive manually to collect
ranging data. Dataset saved to ~/datasets/</span>
      </button>

      <button class="stop-btn" onclick="stopAll()">⬛  STOP ALL</button>
    </div>

    <div id="gamepad">
      <div class="gp-title">🕹 GAMEPAD  <span style="color:#484f58;font-weight:normal">(also WASD / arrow keys)</span></div>
      <div class="gp-grid">
        <div></div>
        <div class="gp-btn" id="gb-up"   onpointerdown="gpDown('w')" onpointerup="gpUp()">↑</div>
        <div></div>
        <div class="gp-btn" id="gb-left" onpointerdown="gpDown('a')" onpointerup="gpUp()">←</div>
        <div class="gp-btn" id="gb-stop" onpointerdown="gpUp()">⬛</div>
        <div class="gp-btn" id="gb-right" onpointerdown="gpDown('d')" onpointerup="gpUp()">→</div>
        <div></div>
        <div class="gp-btn" id="gb-down" onpointerdown="gpDown('s')" onpointerup="gpUp()">↓</div>
        <div></div>
      </div>
      <div class="gp-hint">Hold to move · Release to stop</div>
      <div class="speed-row">
        <span>Speed</span>
        <input type="range" id="spd-lin" min="0.05" max="0.45" step="0.05" value="0.35">
        <span id="spd-val">0.35 m/s</span>
      </div>
    </div>
  </div>

  <div class="right">
    <div class="log-header">OUTPUT LOG</div>
    <div id="log"><span class="info">Launcher ready. Select a mode to begin.</span><br></div>
  </div>
</main>

<script>
let currentMode = null;
let driveKey = null;
let driveInterval = null;
let linSpeed = 0.35;

document.getElementById('spd-lin').addEventListener('input', function() {
  linSpeed = parseFloat(this.value);
  document.getElementById('spd-val').textContent = linSpeed.toFixed(2) + ' m/s';
});

const KEY_VEL = {
  'w': [1, 0], 'ArrowUp': [1, 0],
  's': [-1, 0], 'ArrowDown': [-1, 0],
  'a': [0, 1], 'ArrowLeft': [0, 1],
  'd': [0, -1], 'ArrowRight': [0, -1],
};

document.addEventListener('keydown', e => {
  if(e.repeat) return;
  if(KEY_VEL[e.key]) { e.preventDefault(); gpDown(e.key); }
  if(e.key === ' ' || e.key === 'k') { e.preventDefault(); gpUp(); }
});
document.addEventListener('keyup', e => {
  if(KEY_VEL[e.key]) gpUp();
});

function gpDown(key) {
  if(!currentMode || currentMode === 'autonomous') return;
  driveKey = key;
  const gpMap = {'w':'gb-up','s':'gb-down','a':'gb-left','d':'gb-right',
                 'ArrowUp':'gb-up','ArrowDown':'gb-down','ArrowLeft':'gb-left','ArrowRight':'gb-right'};
  document.querySelectorAll('.gp-btn').forEach(b=>b.classList.remove('pressed'));
  const bid = gpMap[key];
  if(bid) document.getElementById(bid).classList.add('pressed');
  sendDrive(key);
  clearInterval(driveInterval);
  driveInterval = setInterval(() => sendDrive(key), 100);
}
function gpUp() {
  driveKey = null;
  clearInterval(driveInterval);
  document.querySelectorAll('.gp-btn').forEach(b=>b.classList.remove('pressed'));
  fetch('/api/drive', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({linear:0, angular:0})});
}
function sendDrive(key) {
  const [lf, af] = KEY_VEL[key] || [0,0];
  fetch('/api/drive', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({linear: lf * linSpeed, angular: af * 2.0})});
}

function startMode(mode) {
  fetch('/api/start', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({mode})})
  .then(r=>r.json()).then(updateStatus);
}
function stopAll() {
  gpUp();
  fetch('/api/stop', {method:'POST'}).then(r=>r.json()).then(updateStatus);
}

function updateStatus(data) {
  const badge = document.getElementById('badge');
  const label = document.getElementById('mode-label');
  const gamepad = document.getElementById('gamepad');
  const modes = {manual:'MANUAL',autonomous:'AUTONOMOUS',rtt:'RTT COLLECT',stopped:'STOPPED'};
  const classes = {manual:'bm',autonomous:'ba',rtt:'br',stopped:'bs'};
  const m = data.mode || 'stopped';
  badge.className = 'badge ' + (classes[m]||'bs');
  badge.textContent = modes[m] || m.toUpperCase();
  label.textContent = data.detail || '';
  currentMode = m;
  gamepad.style.display = (m==='manual'||m==='rtt') ? 'block' : 'none';
  document.querySelectorAll('.mode-btn').forEach(b=>b.classList.remove('active'));
  const ab = document.getElementById('btn-'+m);
  if(ab) ab.classList.add('active');
}

// Log SSE stream
const logDiv = document.getElementById('log');
const evtSrc = new EventSource('/api/log');
evtSrc.onmessage = e => {
  const line = e.data;
  const span = document.createElement('span');
  if(line.includes('[ERROR]')||line.includes('error')||line.includes('Error'))
    span.className='err';
  else if(line.includes('[INFO]')||line.includes('successfully')||line.includes('active'))
    span.className='ok';
  else if(line.startsWith('>>'))
    span.className='info';
  span.textContent = line;
  logDiv.appendChild(span);
  logDiv.appendChild(document.createElement('br'));
  if(logDiv.children.length > 600) logDiv.removeChild(logDiv.firstChild);
  logDiv.scrollTop = logDiv.scrollHeight;
};

// Poll status every 2 seconds
setInterval(() => {
  fetch('/api/status').then(r=>r.json()).then(updateStatus).catch(()=>{});
}, 2000);
</script>
</body>
</html>
"""


# ── Process manager ───────────────────────────────────────────────────────────

class _ProcManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._procs: dict = {}         # name → Popen
        self._log:  queue.Queue = queue.Queue(maxsize=500)
        self._mode = 'stopped'
        self._detail = ''

    def _log_put(self, line: str):
        try:
            self._log.put_nowait(line)
        except queue.Full:
            try:
                self._log.get_nowait()
                self._log.put_nowait(line)
            except queue.Empty:
                pass

    def _tail(self, name: str, proc):
        for line in proc.stdout:
            l = line.rstrip()
            # Filter very noisy repeated lines
            if 'Timed out waiting for transform' in l:
                continue
            self._log_put(f'[{name}] {l}')
        self._log_put(f'>> Process [{name}] exited.')

    def _start(self, name: str, cmd: str):
        self._log_put(f'>> Starting [{name}]…')
        try:
            proc = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except Exception as e:
            self._log_put(f'[ERROR] Failed to start {name}: {e}')
            return
        with self._lock:
            self._procs[name] = proc
        t = threading.Thread(target=self._tail, args=(name, proc), daemon=True)
        t.start()

    def start_mode(self, mode: str):
        self.stop_all()
        time.sleep(0.5)
        self._log_put(f'>> ─────────────── MODE: {mode.upper()} ───────────────')

        if mode == 'manual':
            self._start('slam', _CMDS['slam'])
            self._mode   = 'manual'
            self._detail = 'SLAM + virtual gamepad active'

        elif mode == 'autonomous':
            self._start('mission', _CMDS['mission'])
            self._mode   = 'autonomous'
            self._detail = 'Dashboard → http://PI_IP:8080'

        elif mode == 'rtt':
            ts = datetime.now().strftime('run_%Y%m%d_%H%M%S')
            out_dir = f'/home/phoenix/datasets/{ts}'
            logger_cmd = (
                f'{_ROS} && mkdir -p {out_dir} && '
                f'ros2 run articubot_one phoenix_logger '
                f'--ros-args '
                f'-p serial_port:=/dev/ttyACM0 '
                f'-p baud:=115200 '
                f'-p output_dir:={out_dir}'
            )
            self._start('slam',   _CMDS['slam'])
            time.sleep(2.0)
            self._start('logger', logger_cmd)
            self._mode   = 'rtt'
            self._detail = f'Saving to ~/datasets/{ts}'
            self._log_put(f'>> RTT dataset directory: {out_dir}')

    def stop_all(self):
        self._log_put('>> Stopping all processes…')
        with self._lock:
            procs = dict(self._procs)
            self._procs.clear()
        for name, proc in procs.items():
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            self._log_put(f'>> [{name}] terminated.')
        # Kill any orphaned ROS processes
        subprocess.run(
            ['bash', '-c',
             'pkill -f ros2_control_node 2>/dev/null; '
             'pkill -f rplidar_composition 2>/dev/null; '
             'pkill -f async_slam_toolbox 2>/dev/null; '
             'pkill -f controller_server 2>/dev/null; '
             'pkill -f camera_detector 2>/dev/null; '
             'pkill -f phoenix_explorer 2>/dev/null; '
             'pkill -f phoenix_dashboard 2>/dev/null; '
             'pkill -f phoenix_logger 2>/dev/null'],
            capture_output=True,
        )
        self._mode   = 'stopped'
        self._detail = ''
        self._log_put('>> All stopped.')

    def status(self):
        with self._lock:
            alive = {n: p.poll() is None for n, p in self._procs.items()}
        return {'mode': self._mode, 'detail': self._detail, 'procs': alive}

    def log_lines(self):
        lines = []
        while True:
            try:
                lines.append(self._log.get_nowait())
            except queue.Empty:
                break
        return lines


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _Handler(http.server.BaseHTTPRequestHandler):
    node:  'PhoenixLauncher'  = None
    pm:    '_ProcManager'     = None

    def do_GET(self):
        path = self.path.split('?')[0]
        try:
            if path == '/':
                self._send_bytes(_HTML.encode('utf-8'), 'text/html; charset=utf-8')
            elif path == '/api/status':
                self._send_json(self.pm.status())
            elif path == '/api/log':
                self._stream_log()
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        path = self.path.split('?')[0]
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length).decode('utf-8') if length else '{}'
        try:
            data = json.loads(body)
        except json.JSONDecodeError:
            data = {}
        try:
            if path == '/api/start':
                mode = data.get('mode', 'manual')
                threading.Thread(
                    target=self.pm.start_mode, args=(mode,), daemon=True
                ).start()
                time.sleep(0.2)
                self._send_json(self.pm.status())

            elif path == '/api/stop':
                threading.Thread(
                    target=self.pm.stop_all, daemon=True
                ).start()
                if self.node:
                    self.node.publish_velocity(0.0, 0.0)
                time.sleep(0.2)
                self._send_json(self.pm.status())

            elif path == '/api/drive':
                linear  = float(data.get('linear',  0.0))
                angular = float(data.get('angular', 0.0))
                if self.node:
                    self.node.publish_velocity(linear, angular)
                self._send_json({'ok': True})

            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_bytes(self, data: bytes, mime: str):
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, obj):
        self._send_bytes(json.dumps(obj).encode(), 'application/json')

    def _stream_log(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                lines = self.pm.log_lines()
                if lines:
                    for line in lines:
                        self.wfile.write(
                            f'data: {line}\n\n'.encode('utf-8'))
                    self.wfile.flush()
                else:
                    time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, *_):
        pass


class _ThreadedServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class PhoenixLauncher(Node):

    def __init__(self):
        super().__init__('phoenix_launcher')
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel_joy', 10)
        self._last_drive = 0.0
        self._linear  = 0.0
        self._angular = 0.0
        # Safety: publish zero if no drive command for 0.6s
        self.create_timer(0.1, self._drive_watchdog)
        self.get_logger().info('Launcher ready — open http://0.0.0.0:8081')

    def publish_velocity(self, linear: float, angular: float):
        self._linear  = float(linear)
        self._angular = float(angular)
        self._last_drive = time.time()
        self._publish_now()

    def _drive_watchdog(self):
        if (self._linear != 0.0 or self._angular != 0.0):
            if time.time() - self._last_drive > 0.6:
                self._linear  = 0.0
                self._angular = 0.0
                self._publish_now()

    def _publish_now(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = self._linear
        msg.twist.angular.z = self._angular
        self._pub.publish(msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PhoenixLauncher()
    pm   = _ProcManager()

    _Handler.node = node
    _Handler.pm   = pm

    server = _ThreadedServer(('0.0.0.0', 8081), _Handler)
    srv_t  = threading.Thread(target=server.serve_forever, daemon=True)
    srv_t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pm.stop_all()
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
