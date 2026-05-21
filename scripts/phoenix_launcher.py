#!/usr/bin/env python3
"""
phoenix_launcher.py — PHOENIX SAR Unified Control + Visualisation UI
=====================================================================

Single web interface at http://<pi-ip>:8081 combining:
  • Mode launcher  (Manual / Autonomous SAR / RTT Collection)
  • Live SLAM map  with robot arrow, start/target markers
  • Camera feed    with detection probability overlay
  • Mission stats  status, elapsed time, target, position
  • Virtual WASD gamepad for manual / RTT driving
  • Compact scrolling log at the bottom

On RTT mode stop, the run folder is completed automatically:
  rtt.csv, trajectory.csv, map.npy, map_meta.json  ← from phoenix_logger
  map.pgm / map.yaml                               ← saved by this node
  (camera video saved separately in ~/ros2_ws/maps/camera_runs/)

Start:
    source ~/ros2_ws/install/setup.bash
    ros2 run articubot_one phoenix_launcher
"""

import http.server
import json
import math
import queue
import re
import shutil
import socketserver
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

import tf2_ros

# ── ROS 2 setup source ────────────────────────────────────────────────────────
_ROS = ('source /opt/ros/jazzy/setup.bash && '
        'source /home/phoenix/ros2_ws/install/setup.bash')

_CMDS = {
    'slam': (f'{_ROS} && ros2 launch articubot_one slam_nav_launch.py '
             'serial_port:=/dev/ttyUSB1'),
    'mission': (f'{_ROS} && ros2 launch articubot_one maze_mission_launch.py '
                'serial_port:=/dev/ttyUSB1 '
                'model_path:=/home/phoenix/model/target_classifier_int8.tflite '
                'metadata_path:=/home/phoenix/model/metadata.json'),
}

_MAX_LOG = 400

# ── Alert rules ───────────────────────────────────────────────────────────────
# (compiled_regex, level, key, human_message)
# level: 'error' | 'warn' | 'ok'
_ALERT_RULES = [
    # XIAO / RTT logger
    (re.compile(r'could not open port /dev/ttyACM0|No such file.*ttyACM0'),
     'error', 'xiao_missing',
     'XIAO not plugged in — RTT data will NOT be collected. Plug in the XIAO ESP32-S3.'),
    (re.compile(r'Serial open: /dev/ttyACM0'),
     'ok', 'xiao_ok', 'XIAO connected — RTT ranging active'),

    # LIDAR
    (re.compile(r'RPLIDAR S/N:'),
     'ok', 'lidar_ok', 'LIDAR online and scanning'),
    (re.compile(r'Failed to activate local_costmap because transform from base_link to map'),
     'error', 'lidar_no_tf',
     'LIDAR not scanning — USB ports probably swapped. Unplug both USB cables, replug ESP32 first, then LIDAR.'),

    # Nav2
    (re.compile(r'Aborting bringup'),
     'warn', 'nav2_abort', 'Nav2 failed to start (caused by LIDAR issue above)'),
    (re.compile(r'lifecycle_manager_navigation.*Managed nodes are active'),
     'ok', 'nav2_ok', 'SLAM + Nav2 fully active'),

    # Motor controller
    (re.compile(r'DiffDriveArduinoHardware.*Successfully activated'),
     'ok', 'motors_ok', 'Motor controller connected'),
    (re.compile(r'ReadByte.*timeout', re.IGNORECASE),
     'warn', 'esp32_timeout',
     'ESP32 motor timeout — power cycle the ESP32 USB cable'),
    (re.compile(r'could not open port /dev/ttyUSB0|No such file.*ttyUSB0'),
     'error', 'motors_missing',
     'ESP32 motor controller not found on /dev/ttyUSB0'),

    # Camera
    (re.compile(r'rpicam-vid.*[Ee]rror|[Cc]annot open.*camera|no devices found', re.IGNORECASE),
     'error', 'camera_error',
     'Camera failed — check Pi Camera ribbon cable'),
]


# ── Embedded HTML/CSS/JS ──────────────────────────────────────────────────────

_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PHOENIX SAR</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Courier New',monospace;
     height:100vh;display:flex;flex-direction:column;overflow:hidden}

/* ── Header ── */
header{background:#161b22;border-bottom:1px solid #30363d;padding:7px 16px;
       display:flex;align-items:center;gap:12px;flex-shrink:0;min-height:42px}
header h1{color:#58a6ff;font-size:.95rem;letter-spacing:3px;white-space:nowrap}
.badge{padding:2px 9px;border-radius:9px;font-size:.65rem;font-weight:bold;letter-spacing:1px;white-space:nowrap}
.bs{background:#21262d;color:#8b949e;border:1px solid #30363d}
.bm{background:#0d419d;color:#79c0ff;border:1px solid #1f6feb}
.ba{background:#033a16;color:#3fb950;border:1px solid #238636}
.br{background:#4d2d00;color:#e3b341;border:1px solid #9e6a03}
.hstat{font-size:.72rem;color:#8b949e;white-space:nowrap}
.hstat span{color:#c9d1d9}
.prob-wrap{margin-left:auto;display:flex;align-items:center;gap:7px;font-size:.72rem}
.pb-bg{width:90px;height:6px;background:#21262d;border-radius:3px;overflow:hidden}
.pb-fill{height:100%;background:#238636;border-radius:3px;transition:width .3s}

/* ── Main 3-column grid ── */
main{flex:1;display:grid;grid-template-columns:250px 1fr 295px;
     gap:1px;background:#30363d;min-height:0;overflow:hidden}
.right-col{display:grid;grid-template-rows:1fr 195px;gap:1px;background:#30363d;min-height:0}

/* ── Panels ── */
.panel{background:#0d1117;display:flex;flex-direction:column;overflow:hidden;min-height:0}
.ph{background:#161b22;border-bottom:1px solid #30363d;padding:4px 10px;
    font-size:.6rem;letter-spacing:2px;color:#58a6ff;font-weight:bold;flex-shrink:0}
.pc{flex:1;overflow:hidden;display:flex;align-items:center;justify-content:center;min-height:0}

/* ── Controls column ── */
.ctrl-body{flex:1;overflow-y:auto;padding:10px 10px 6px;display:flex;flex-direction:column;gap:6px}
.mode-btn{width:100%;padding:10px 10px;border-radius:6px;border:1px solid #30363d;
  background:#161b22;color:#c9d1d9;font-family:inherit;font-size:.78rem;cursor:pointer;
  text-align:left;transition:all .15s}
.mode-btn:hover{border-color:#58a6ff;background:#1c2333}
.mode-btn.active{border-color:#238636;background:#0d2818}
.mode-btn .mn{font-weight:bold;font-size:.82rem;display:block;margin-bottom:2px}
.mode-btn .md{color:#8b949e;font-size:.65rem;line-height:1.4}
.stop-btn{width:100%;padding:9px;border-radius:6px;border:1px solid #da3633;
  background:#21262d;color:#f85149;font-family:inherit;font-size:.82rem;
  font-weight:bold;cursor:pointer;letter-spacing:1px;transition:all .15s;margin-top:2px}
.stop-btn:hover{background:#4d1010;border-color:#f85149}

/* ── Gamepad ── */
#gamepad{display:none;padding:8px 10px;border-top:1px solid #30363d;flex-shrink:0}
.gp-t{font-size:.6rem;color:#e3b341;letter-spacing:2px;font-weight:bold;margin-bottom:7px}
.gp-grid{display:grid;grid-template-columns:repeat(3,52px);grid-template-rows:repeat(3,52px);
  gap:5px;justify-content:center}
.gp-btn{background:#21262d;border:1px solid #30363d;border-radius:7px;color:#c9d1d9;
  font-size:1.1rem;cursor:pointer;user-select:none;display:flex;align-items:center;
  justify-content:center;transition:all .1s;-webkit-tap-highlight-color:transparent;touch-action:none}
.gp-btn:active,.gp-btn.pressed{background:#0d419d;border-color:#58a6ff;transform:scale(.92)}
.gp-hint{text-align:center;font-size:.6rem;color:#484f58;margin-top:5px}
.spd-row{display:flex;align-items:center;gap:6px;margin-top:7px;font-size:.68rem;color:#8b949e}
.spd-row input{flex:1;accent-color:#58a6ff}

/* ── Map ── */
#map-img{max-width:100%;max-height:100%;image-rendering:pixelated;image-rendering:crisp-edges}

/* ── Camera ── */
#cam-img{max-width:100%;max-height:100%;object-fit:contain}

/* ── Stats ── */
.stats-body{flex:1;overflow-y:auto;padding:6px 9px;display:flex;flex-direction:column;gap:5px}
.stat-row{display:grid;grid-template-columns:1fr 1fr;gap:5px}
.sc{background:#161b22;border:1px solid #30363d;border-radius:4px;padding:5px 8px}
.sl{color:#8b949e;font-size:.55rem;letter-spacing:1px;text-transform:uppercase;margin-bottom:1px}
.sv{color:#c9d1d9;font-size:.75rem;font-weight:bold}
.sv.g{color:#3fb950}.sv.y{color:#e3b341}.sv.r{color:#f85149}
.prob-bar{margin:0 9px 5px;height:2px;background:#21262d;border-radius:1px;overflow:hidden}
.prob-bar-fill{height:100%;background:linear-gradient(90deg,#1f6feb,#3fb950);transition:width .4s}

/* ── Log footer ── */
footer{height:90px;background:#0d1117;border-top:1px solid #30363d;display:flex;flex-direction:column;flex-shrink:0}
.fh{background:#161b22;border-bottom:1px solid #30363d;padding:3px 10px;
    font-size:.58rem;letter-spacing:2px;color:#58a6ff;font-weight:bold}
#log{flex:1;overflow-y:auto;padding:4px 10px;font-size:.65rem;line-height:1.5;color:#6e7681}
#log .err{color:#f85149}#log .ok{color:#3fb950}#log .inf{color:#58a6ff}

/* ── Alert bar ── */
#alerts{flex-shrink:0;display:flex;flex-direction:column;gap:2px;padding:0}
#alerts:empty{display:none}
.al{display:flex;align-items:center;gap:8px;padding:5px 12px;font-size:.72rem;
    border-left:4px solid;animation:alin .2s}
@keyframes alin{from{opacity:0;transform:translateY(-3px)}to{opacity:1;transform:none}}
.al.err{background:#2d0a0a;border-color:#f85149;color:#ffa198}
.al.wrn{background:#2d1e00;border-color:#e3b341;color:#f0c000}
.al.ok {background:#0d2818;border-color:#3fb950;color:#56d364}
.al .ali{font-size:.85rem;flex-shrink:0}
.al .alm{flex:1}
.al .alx{cursor:pointer;padding:0 4px;opacity:.5;font-size:.9rem;flex-shrink:0}
.al .alx:hover{opacity:1}
</style>
</head>
<body>
<header>
  <h1>PHOENIX SAR</h1>
  <span id="badge" class="badge bs">STOPPED</span>
  <div class="hstat">TIME&nbsp;<span id="h-time">--</span></div>
  <div class="hstat">POS&nbsp;<span id="h-pos">--</span></div>
  <div class="prob-wrap">
    <span id="h-pct">0%</span>
    <div class="pb-bg"><div id="h-pb" class="pb-fill" style="width:0%"></div></div>
  </div>
</header>

<div id="alerts"></div>

<main>
  <!-- ── Controls ── -->
  <div class="panel">
    <div class="ph">MODES</div>
    <div class="ctrl-body">
      <button class="mode-btn" id="btn-manual" onclick="startMode('manual')">
        <span class="mn">🎮  MANUAL CONTROL</span>
        <span class="md">WASD gamepad + SLAM mapping</span>
      </button>
      <button class="mode-btn" id="btn-autonomous" onclick="startMode('autonomous')">
        <span class="mn">🤖  AUTONOMOUS MISSION</span>
        <span class="md">SAR frontier exploration + camera detection</span>
      </button>
      <button class="mode-btn" id="btn-rtt" onclick="startMode('rtt')">
        <span class="mn">📡  RTT DATA COLLECT</span>
        <span class="md">SLAM + FTM logger · all files auto-saved</span>
      </button>
      <button class="stop-btn" onclick="stopAll()">⬛  STOP ALL</button>
    </div>

    <div id="gamepad">
      <div class="gp-t">🕹 GAMEPAD · WASD / ARROWS</div>
      <div class="gp-grid">
        <div></div>
        <div class="gp-btn" id="gb-u" onpointerdown="gpDown('w')" onpointerup="gpUp()" onpointerleave="gpUp()">↑</div>
        <div></div>
        <div class="gp-btn" id="gb-l" onpointerdown="gpDown('a')" onpointerup="gpUp()" onpointerleave="gpUp()">←</div>
        <div class="gp-btn"          onpointerdown="gpUp()">⬛</div>
        <div class="gp-btn" id="gb-r" onpointerdown="gpDown('d')" onpointerup="gpUp()" onpointerleave="gpUp()">→</div>
        <div></div>
        <div class="gp-btn" id="gb-d" onpointerdown="gpDown('s')" onpointerup="gpUp()" onpointerleave="gpUp()">↓</div>
        <div></div>
      </div>
      <div class="gp-hint">Hold = move · Release = stop · Space = e-stop</div>
      <div class="spd-row">
        <span>Speed</span>
        <input type="range" id="spd" min="0.05" max="0.45" step="0.05" value="0.35">
        <span id="spd-v">0.35 m/s</span>
      </div>
    </div>
  </div>

  <!-- ── SLAM Map ── -->
  <div class="panel">
    <div class="ph">SLAM MAP</div>
    <div class="pc" style="background:#0a0c0f;padding:2px">
      <img id="map-img" src="/map.png" alt="">
    </div>
  </div>

  <!-- ── Right column: camera + stats ── -->
  <div class="right-col">
    <div class="panel">
      <div class="ph">CAMERA FEED</div>
      <div class="pc" style="background:#000;padding:0">
        <img id="cam-img" src="/camera" alt="">
      </div>
    </div>
    <div class="panel">
      <div class="ph">MISSION</div>
      <div class="prob-bar"><div id="prob-fill" class="prob-bar-fill" style="width:0%"></div></div>
      <div class="stats-body">
        <div class="stat-row">
          <div class="sc"><div class="sl">Status</div><div id="s-status" class="sv">STOPPED</div></div>
          <div class="sc"><div class="sl">Elapsed</div><div id="s-elapsed" class="sv">--</div></div>
        </div>
        <div class="stat-row">
          <div class="sc"><div class="sl">Probability</div><div id="s-prob" class="sv">--</div></div>
          <div class="sc"><div class="sl">Target</div><div id="s-target" class="sv">--</div></div>
        </div>
        <div class="stat-row">
          <div class="sc"><div class="sl">Robot Pos</div><div id="s-robot" class="sv">--</div></div>
          <div class="sc"><div class="sl">Start Pos</div><div id="s-start" class="sv">--</div></div>
        </div>
        <div class="stat-row">
          <div class="sc" style="grid-column:span 2">
            <div class="sl">Target Position</div>
            <div id="s-final" class="sv">--</div>
          </div>
        </div>
      </div>
    </div>
  </div>
</main>

<footer>
  <div class="fh">LOG</div>
  <div id="log"><span class="inf">Launcher ready.</span><br></div>
</footer>

<script>
const BC={manual:'bm',autonomous:'ba',rtt:'br',stopped:'bs'};
const BL={manual:'MANUAL',autonomous:'AUTONOMOUS',rtt:'RTT COLLECT',stopped:'STOPPED'};

let curMode=null, linSpeed=0.35;
document.getElementById('spd').addEventListener('input',function(){
  linSpeed=parseFloat(this.value);
  document.getElementById('spd-v').textContent=linSpeed.toFixed(2)+' m/s';
});

const KV={'w':[1,0],'ArrowUp':[1,0],'s':[-1,0],'ArrowDown':[-1,0],
          'a':[0,1],'ArrowLeft':[0,1],'d':[0,-1],'ArrowRight':[0,-1]};
document.addEventListener('keydown',e=>{if(e.repeat)return;if(KV[e.key]){e.preventDefault();gpDown(e.key);}if(e.key===' '||e.key==='k'){e.preventDefault();gpUp();}});
document.addEventListener('keyup',e=>{if(KV[e.key])gpUp();});

let driveIv=null;
function gpDown(k){
  if(!curMode||curMode==='autonomous')return;
  document.querySelectorAll('.gp-btn').forEach(b=>b.classList.remove('pressed'));
  const m={'w':'gb-u','s':'gb-d','a':'gb-l','d':'gb-r','ArrowUp':'gb-u','ArrowDown':'gb-d','ArrowLeft':'gb-l','ArrowRight':'gb-r'};
  if(m[k])document.getElementById(m[k]).classList.add('pressed');
  clearInterval(driveIv);
  sendD(k); driveIv=setInterval(()=>sendD(k),100);
}
function gpUp(){
  clearInterval(driveIv);
  document.querySelectorAll('.gp-btn').forEach(b=>b.classList.remove('pressed'));
  post('/api/drive',{linear:0,angular:0});
}
function sendD(k){
  const [l,a]=KV[k]||[0,0];
  post('/api/drive',{linear:l*linSpeed,angular:a*2.0});
}

function startMode(m){post('/api/start',{mode:m}).then(updateUI);}
function stopAll(){gpUp();post('/api/stop').then(updateUI);}

function post(url,body){
  return fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body||{})}).then(r=>r.json()).catch(()=>({}));
}
function fp(p){return p?'('+p.x.toFixed(2)+','+p.y.toFixed(2)+')':'--';}
function ft(s){if(!s&&s!==0)return'--';const m=Math.floor(s/60),r=s%60;return m?m+'m '+r+'s':s+'s';}

function updateUI(d){
  if(!d||!d.mode)return;
  const m=d.mode;
  const b=document.getElementById('badge');
  b.textContent=BL[m]||m.toUpperCase(); b.className='badge '+(BC[m]||'bs');
  document.getElementById('gamepad').style.display=(m==='manual'||m==='rtt')?'block':'none';
  document.querySelectorAll('.mode-btn').forEach(x=>x.classList.remove('active'));
  const ab=document.getElementById('btn-'+m); if(ab)ab.classList.add('active');
  curMode=m;
}

function pollStats(){
  fetch('/api/stats').then(r=>r.json()).then(d=>{
    const pct=Math.round(d.prob*100);
    document.getElementById('h-time').textContent=ft(d.elapsed);
    document.getElementById('h-pos').textContent=fp(d.robot);
    document.getElementById('h-pct').textContent=pct+'%';
    document.getElementById('h-pb').style.width=pct+'%';
    const ss=document.getElementById('s-status');
    ss.textContent=d.status;
    ss.className='sv '+(d.status==='TARGET_FOUND'||d.status==='MISSION_COMPLETE'?'g':d.status==='SPINNING'?'y':'');
    document.getElementById('s-elapsed').textContent=ft(d.elapsed);
    const sp=document.getElementById('s-prob');
    sp.textContent=pct+'%'; sp.className='sv '+(pct>70?'g':pct>40?'y':'');
    const st=document.getElementById('s-target');
    st.textContent=d.detected?'FOUND ★':'Scanning'; st.className='sv '+(d.detected?'g':'');
    document.getElementById('s-robot').textContent=fp(d.robot);
    document.getElementById('s-start').textContent=fp(d.start);
    document.getElementById('s-final').textContent=d.final?fp(d.final)+(d.detected?' — TARGET':''):'--';
    document.getElementById('prob-fill').style.width=pct+'%';
    // also update mode badge from stats
    const b=document.getElementById('badge');
    if(d.launcher_mode){
      b.textContent=BL[d.launcher_mode]||d.launcher_mode.toUpperCase();
      b.className='badge '+(BC[d.launcher_mode]||'bs');
      curMode=d.launcher_mode;
      document.getElementById('gamepad').style.display=(curMode==='manual'||curMode==='rtt')?'block':'none';
    }
    // refresh map
    document.getElementById('map-img').src='/map.png?t='+Date.now();
  }).catch(()=>{});
}
setInterval(pollStats,1000); pollStats();

// ── Alert system ──
const ALERT_ICONS={error:'✕',warn:'⚠',ok:'✓'};
const ALERT_CLEARS={xiao_ok:['xiao_missing'],lidar_ok:['lidar_no_tf','nav2_abort'],
                    motors_ok:['motors_missing','esp32_timeout'],nav2_ok:['nav2_abort']};
const alertsDiv=document.getElementById('alerts');

function showAlert(level,key,msg){
  // clear any superseded alerts
  (ALERT_CLEARS[key]||[]).forEach(k=>{
    const old=document.getElementById('al-'+k); if(old)old.remove();
  });
  // remove existing same-key alert
  const ex=document.getElementById('al-'+key); if(ex)ex.remove();
  const el=document.createElement('div');
  const cls={error:'err',warn:'wrn',ok:'ok'}[level]||'ok';
  el.className='al '+cls; el.id='al-'+key;
  el.innerHTML=`<span class="ali">${ALERT_ICONS[level]||'•'}</span>`+
               `<span class="alm">${msg}</span>`+
               `<span class="alx" onclick="this.parentElement.remove()">×</span>`;
  alertsDiv.appendChild(el);
  if(level==='ok') setTimeout(()=>el.remove(),7000);
}

function clearAlerts(){ alertsDiv.innerHTML=''; }

// ── SSE log stream ──
const es=new EventSource('/api/log');
const logDiv=document.getElementById('log');
es.onmessage=e=>{
  const t=e.data;
  // handle alert signals
  if(t.startsWith('ALERT:')){
    const parts=t.split(':');
    const level=parts[1], key=parts[2], msg=parts.slice(3).join(':');
    if(level==='clear'){ clearAlerts(); return; }
    showAlert(level,key,msg); return;
  }
  // normal log line
  const s=document.createElement('span');
  if(t.includes('[ERROR]')||t.includes('error')||t.includes('Error'))s.className='err';
  else if(t.includes('[INFO]')||t.includes('active')||t.includes('successfully')||t.startsWith('>>'))s.className=t.startsWith('>>')?'inf':'ok';
  s.textContent=t;
  logDiv.appendChild(s); logDiv.appendChild(document.createElement('br'));
  if(logDiv.children.length>500)logDiv.removeChild(logDiv.firstChild);
  logDiv.scrollTop=logDiv.scrollHeight;
};
</script>
</body>
</html>
"""


# ── Process manager ───────────────────────────────────────────────────────────

class _ProcManager:
    def __init__(self):
        self._lock      = threading.Lock()
        self._mode_lock = threading.Lock()   # prevents concurrent start/stop
        self._procs: dict = {}
        self._log:   queue.Queue = queue.Queue(maxsize=600)
        self._mode   = 'stopped'
        self._rtt_dir: str = ''
        self._alerted: set = set()           # alert keys fired this session

    def _put(self, line: str):
        try:
            self._log.put_nowait(line)
        except queue.Full:
            try:
                self._log.get_nowait()
                self._log.put_nowait(line)
            except queue.Empty:
                pass

    def _check_alerts(self, line: str):
        for pattern, level, key, msg in _ALERT_RULES:
            if key in self._alerted:
                continue
            if pattern.search(line):
                self._alerted.add(key)
                self._put(f'ALERT:{level}:{key}:{msg}')
                break

    def _tail(self, name: str, proc):
        for line in proc.stdout:
            l = line.rstrip()
            if 'Timed out waiting for transform' in l:
                continue
            self._put(f'[{name}] {l}')
            self._check_alerts(l)
        self._put(f'>> [{name}] exited.')

    def _launch(self, name: str, cmd: str):
        self._put(f'>> Starting [{name}]…')
        try:
            proc = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except Exception as e:
            self._put(f'[ERROR] {name}: {e}')
            return
        with self._lock:
            self._procs[name] = proc
        threading.Thread(target=self._tail, args=(name, proc), daemon=True).start()

    def start_mode(self, mode: str):
        # Drop duplicate requests (e.g. double-click) — only one mode change at a time
        if not self._mode_lock.acquire(blocking=False):
            self._put('>> Mode change already in progress — ignoring duplicate.')
            return
        try:
            self.stop_all()
            time.sleep(2.5)   # wait for ports/PIDs to fully release
            self._alerted.clear()
            self._put('ALERT:clear::')
            self._put(f'>> ─── MODE: {mode.upper()} ───')

            # Disable DTR/RTS hangup on ESP32 serial port to prevent reset-on-open
            subprocess.run(['stty', '-F', '/dev/ttyUSB0', '-hupcl', 'clocal'],
                           capture_output=True)

            if mode == 'manual':
                self._launch('slam', _CMDS['slam'])
                self._mode = 'manual'

            elif mode == 'autonomous':
                self._launch('mission', _CMDS['mission'])
                self._mode = 'autonomous'

            elif mode == 'rtt':
                ts  = datetime.now().strftime('run_%Y%m%d_%H%M%S')
                out = f'/home/phoenix/datasets/{ts}'
                self._rtt_dir = out
                logger_cmd = (
                    f'{_ROS} && mkdir -p {out} && '
                    f'ros2 run articubot_one phoenix_logger '
                    f'--ros-args '
                    f'-p serial_port:=/dev/ttyACM0 '
                    f'-p baud:=115200 '
                    f'-p output_dir:={out}'
                )
                self._launch('slam', _CMDS['slam'])
                time.sleep(2.0)
                self._launch('logger', logger_cmd)
                self._mode = 'rtt'
                self._put(f'>> RTT run directory: {out}')
        finally:
            self._mode_lock.release()

    def stop_all(self, save_rtt_map: bool = True):
        self._put('>> Stopping all…')
        rtt_dir = self._rtt_dir if (self._mode == 'rtt' and save_rtt_map) else ''

        # Save map BEFORE killing SLAM (map_saver_cli needs /map topic alive)
        if rtt_dir:
            self._put(f'>> Saving SLAM map to {rtt_dir}/ …')
            try:
                result = subprocess.run(
                    ['bash', '-c',
                     f'{_ROS} && ros2 run nav2_map_server map_saver_cli '
                     f'-f {rtt_dir}/slam_map '
                     f'--ros-args -p save_map_timeout:=5.0'],
                    capture_output=True, text=True, timeout=15,
                )
                if result.returncode == 0:
                    self._put(f'>> Map saved: {rtt_dir}/slam_map.pgm/.yaml')
                else:
                    self._put('>> Map save failed')
            except Exception as e:
                self._put(f'>> Map save error: {e}')

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
        # kill orphans
        subprocess.run(['bash', '-c',
            'pkill -f ros2_control_node 2>/dev/null; '
            'pkill -f rplidar_composition 2>/dev/null; '
            'pkill -f async_slam_toolbox 2>/dev/null; '
            'pkill -f controller_server 2>/dev/null; '
            'pkill -f camera_detector 2>/dev/null; '
            'pkill -f phoenix_explorer 2>/dev/null; '
            'pkill -f phoenix_dashboard 2>/dev/null; '
            'pkill -f phoenix_logger 2>/dev/null'],
            capture_output=True)

        if rtt_dir:
            self._put(f'>> ─── RTT run complete ───')
            self._put(f'>> Folder  : {rtt_dir}/')
            self._put( '>> Files   : rtt.csv, trajectory.csv, map.npy, map_meta.json, slam_map.pgm/.yaml')
            self._put( '>> Camera  : ~/ros2_ws/maps/camera_runs/  (autonomous mode only)')
            self._put(f'>> Transfer: scp -r phoenix@PI:{rtt_dir} .')

        self._mode    = 'stopped'
        self._rtt_dir = ''
        self._put('>> All stopped.')

    @property
    def mode(self): return self._mode

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
    node: 'PhoenixLauncher' = None
    pm:   '_ProcManager'    = None

    def do_GET(self):
        path = self.path.split('?')[0]
        try:
            if   path == '/':         self._bytes(_HTML.encode('utf-8'), 'text/html; charset=utf-8')
            elif path == '/map.png':  self._bytes(self.node.gen_map(), 'image/png', cache=False)
            elif path == '/camera':   self._mjpeg()
            elif path == '/api/stats':self._bytes(self.node.get_stats(self.pm.mode).encode(), 'application/json', cache=False)
            elif path == '/api/log':  self._sse()
            else:                     self.send_error(404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        path = self.path.split('?')[0]
        n = int(self.headers.get('Content-Length', 0))
        try:
            data = json.loads(self.rfile.read(n).decode()) if n else {}
        except json.JSONDecodeError:
            data = {}
        try:
            if path == '/api/start':
                mode = data.get('mode', 'manual')
                if mode in ('manual', 'rtt'):
                    threading.Thread(target=self.node.start_raw_camera,
                                     daemon=True).start()
                else:
                    self.node.stop_raw_camera()
                threading.Thread(target=self.pm.start_mode, args=(mode,),
                                 daemon=True).start()
                time.sleep(0.3)
                self._bytes(json.dumps({'mode': self.pm.mode}).encode(), 'application/json')
            elif path == '/api/stop':
                self.node.stop_raw_camera()
                threading.Thread(target=self.pm.stop_all, daemon=True).start()
                if self.node: self.node.publish_velocity(0.0, 0.0)
                time.sleep(0.3)
                self._bytes(json.dumps({'mode': self.pm.mode}).encode(), 'application/json')
            elif path == '/api/drive':
                if self.node:
                    self.node.publish_velocity(
                        float(data.get('linear', 0.0)),
                        float(data.get('angular', 0.0)))
                self._bytes(b'{"ok":1}', 'application/json')
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _bytes(self, data: bytes, mime: str, cache: bool = True):
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        if not cache:
            self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        offline = PhoenixLauncher._offline_jpg()
        prev = None
        while True:
            try:
                with self.node._lock:
                    jpg = self.node._frame_jpg
                if jpg is None:
                    jpg = offline
                if jpg is not prev:
                    prev = jpg
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                    self.wfile.flush()
                else:
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    def _sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                lines = self.pm.log_lines()
                if lines:
                    for l in lines:
                        self.wfile.write(f'data: {l}\n\n'.encode('utf-8'))
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

        # ── state ──────────────────────────────────────────────────────────────
        self._lock      = threading.Lock()
        self._map_msg   = None
        self._frame_jpg = None
        self._detected  = False
        self._prob      = 0.0
        self._status    = 'WAITING'
        self._elapsed   = 0
        self._robot_xyz = None
        self._start_xy  = None
        self._final_xy  = None
        self._explore_t0 = None

        # ── velocity publisher ─────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(TwistStamped, '/cmd_vel_joy', 10)
        self._lin = 0.0
        self._ang = 0.0
        self._last_cmd  = 0.0
        self.create_timer(0.1, self._drive_watchdog)

        # ── raw camera (for manual/RTT when camera_detector isn't running) ──
        self._raw_cam_proc   = None
        self._raw_cam_active = False
        self._ui_log = lambda msg: None  # replaced by pm._put after init

        # ── TF ─────────────────────────────────────────────────────────────────
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ── subscriptions ──────────────────────────────────────────────────────
        map_qos = QoSProfile(depth=1)
        map_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid,   '/map',                    self._on_map,      map_qos)
        self.create_subscription(CompressedImage, '/phoenix/camera_frame',    self._on_frame,    1)
        self.create_subscription(Bool,            '/phoenix/target_detected', self._on_detected, 10)
        self.create_subscription(Float32,         '/phoenix/detection_prob',  self._on_prob,     10)
        self.create_subscription(String,          '/phoenix/status',          self._on_status,   10)
        self.create_subscription(PoseStamped,     '/phoenix/start_pose',      self._on_start,    10)
        self.create_subscription(PoseStamped,     '/phoenix/final_pose',      self._on_final,    10)

        self.create_timer(0.5, self._update_tf)
        self.get_logger().info('Launcher ready — http://0.0.0.0:8081')

    # ── velocity ───────────────────────────────────────────────────────────────

    def publish_velocity(self, linear: float, angular: float):
        self._lin = float(linear)
        self._ang = float(angular)
        self._last_cmd = time.time()
        self._pub_vel()

    def _drive_watchdog(self):
        if (self._lin != 0.0 or self._ang != 0.0):
            if time.time() - self._last_cmd > 0.6:
                self._lin = 0.0; self._ang = 0.0
                self._pub_vel()

    def _pub_vel(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = self._lin
        msg.twist.angular.z = self._ang
        self._cmd_pub.publish(msg)

    # ── ROS callbacks ──────────────────────────────────────────────────────────

    def _on_map(self, msg):
        with self._lock:
            self._map_msg = msg

    def _on_frame(self, msg):
        arr   = np.frombuffer(bytes(msg.data), np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            prob, det = self._prob, self._detected
        self._draw_overlay(frame, prob, det)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok:
            with self._lock:
                self._frame_jpg = buf.tobytes()

    def _on_detected(self, msg):
        with self._lock:
            self._detected = bool(msg.data)

    def _on_prob(self, msg):
        with self._lock:
            self._prob = float(msg.data)

    def _on_status(self, msg):
        with self._lock:
            self._status = msg.data
            if msg.data == 'EXPLORING' and self._explore_t0 is None:
                self._explore_t0 = time.time()

    def _on_start(self, msg):
        with self._lock:
            self._start_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_final(self, msg):
        with self._lock:
            self._final_xy = (msg.pose.position.x, msg.pose.position.y)

    def _update_tf(self):
        try:
            tf  = self.tf_buf.lookup_transform('map', 'base_link', Time(),
                                               timeout=Duration(seconds=0.1))
            t   = tf.transform.translation
            r   = tf.transform.rotation
            yaw = math.atan2(2.0*(r.w*r.z+r.x*r.y), 1.0-2.0*(r.y*r.y+r.z*r.z))
            with self._lock:
                self._robot_xyz = (t.x, t.y, yaw)
                if self._explore_t0 is not None:
                    self._elapsed = int(time.time() - self._explore_t0)
        except Exception:
            pass

    # ── camera overlay ─────────────────────────────────────────────────────────

    @staticmethod
    def _draw_overlay(frame, prob: float, detected: bool):
        h, w = frame.shape[:2]
        bc = (30, 200, 80) if detected else (30, 30, 30)
        tc = (50, 255, 100) if detected else (180, 180, 180)
        cv2.rectangle(frame, (0, 0), (w, 28), bc, -1)
        lbl = f'TARGET FOUND  {prob*100:.0f}%' if detected else f'Scanning...  {prob*100:.0f}%'
        cv2.putText(frame, lbl, (7, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55, tc, 2, cv2.LINE_AA)
        cv2.rectangle(frame, (0, h-5), (w, h), (40, 40, 40), -1)
        fw = int(w * max(0.0, min(1.0, prob)))
        cv2.rectangle(frame, (0, h-5), (fw, h), (50, 200, 80) if detected else (50, 140, 220), -1)
        if detected:
            cv2.rectangle(frame, (2, 2), (w-2, h-2), (50, 255, 80), 3)

    @staticmethod
    def _offline_jpg() -> bytes:
        img = np.full((200, 280, 3), 15, dtype=np.uint8)
        cv2.putText(img, 'CAMERA OFFLINE', (30, 108),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (50, 50, 50), 1, cv2.LINE_AA)
        _, buf = cv2.imencode('.jpg', img)
        return buf.tobytes()

    # ── map rendering ──────────────────────────────────────────────────────────

    # ── raw camera streaming (no TFLite — used in manual / RTT modes) ────────

    def start_raw_camera(self):
        """Open rpicam-vid directly when camera_detector isn't running."""
        self.stop_raw_camera()
        rpicam = shutil.which('rpicam-vid')
        if rpicam is None:
            for p in ('/usr/bin/rpicam-vid', '/usr/local/bin/rpicam-vid'):
                if Path(p).exists():
                    rpicam = p
                    break
        if not rpicam:
            msg = '>> Camera: rpicam-vid not found — check Pi Camera is enabled'
            self._ui_log(msg)
            self.get_logger().warn(msg)
            return
        try:
            self._raw_cam_proc = subprocess.Popen(
                [rpicam, '-t', '0', '--codec', 'mjpeg', '-o', '-',
                 '--width', '640', '--height', '480', '--framerate', '10',
                 '--nopreview', '--flush'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0,
            )
        except Exception as e:
            msg = f'>> Camera: rpicam-vid launch failed — {e}'
            self._ui_log(msg)
            self.get_logger().warn(msg)
            return
        self._raw_cam_active = True
        threading.Thread(target=self._raw_cam_reader, daemon=True).start()
        threading.Thread(target=self._raw_cam_stderr, daemon=True).start()
        self._ui_log('>> Camera stream started.')

    def stop_raw_camera(self):
        self._raw_cam_active = False
        if self._raw_cam_proc is not None:
            try:
                self._raw_cam_proc.terminate()
            except Exception:
                pass
            self._raw_cam_proc = None
        # Kill any orphan rpicam-vid from a previous launcher session
        subprocess.run(['pkill', '-f', 'rpicam-vid'], capture_output=True)

    # Noisy V4L2 buffer warnings that appear under load — not user-actionable
    _CAM_NOISE = re.compile(
        r'Failed to queue buffer|RPISTREAM|V4L2 pixel format|'
        r'libcamera v|libpisp|pisp\.cpp|Using tuning file|'
        r'Adding camera|Registered camera|Resizing costmap|'
        r'Mode selection|Score:|Stream configuration|configuring streams|'
        r'Selected sensor|Halting:|INFO |WARN ')

    def _raw_cam_stderr(self):
        """Forward rpicam-vid fatal errors to the web UI log (filters V4L2 noise)."""
        try:
            for line in self._raw_cam_proc.stderr:
                l = line.rstrip().decode('utf-8', errors='replace')
                if l and not self._CAM_NOISE.search(l):
                    self._ui_log(f'[camera] {l}')
        except Exception:
            pass

    def _raw_cam_reader(self):
        SOI = b'\xff\xd8'
        EOI = b'\xff\xd9'
        buf = b''
        while self._raw_cam_active and self._raw_cam_proc is not None:
            try:
                chunk = self._raw_cam_proc.stdout.read(8192)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
            while True:
                s = buf.find(SOI)
                if s == -1:
                    buf = b''
                    break
                e = buf.find(EOI, s + 2)
                if e == -1:
                    buf = buf[s:]
                    break
                jpeg = buf[s:e + 2]
                buf  = buf[e + 2:]
                arr  = np.frombuffer(jpeg, np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
                    h, w = frame.shape[:2]
                    cv2.rectangle(frame, (0, 0), (w, 26), (15, 15, 40), -1)
                    cv2.putText(frame, 'CAMERA LIVE', (8, 18),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (100, 200, 255), 1, cv2.LINE_AA)
                    ok, buf2 = cv2.imencode('.jpg', frame,
                                           [cv2.IMWRITE_JPEG_QUALITY, 72])
                    if ok:
                        with self._lock:
                            self._frame_jpg = buf2.tobytes()

    def gen_map(self) -> bytes:
        with self._lock:
            map_msg = self._map_msg
            robot   = self._robot_xyz
            start   = self._start_xy
            final   = self._final_xy

        if map_msg is None:
            img = np.full((180, 280, 3), 13, dtype=np.uint8)
            cv2.putText(img, 'Waiting for SLAM map...', (14, 96),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (60, 60, 60), 1, cv2.LINE_AA)
            _, buf = cv2.imencode('.png', img)
            return buf.tobytes()

        info = map_msg.info
        grid = np.array(map_msg.data, dtype=np.int8).reshape(info.height, info.width)
        img  = np.full((info.height, info.width, 3), 55, dtype=np.uint8)
        img[(grid >= 0) & (grid <= 50)] = [220, 220, 220]
        img[grid > 50]                  = [22,  22,  22]

        max_dim = max(info.width, info.height, 1)
        scale   = max(1, min(8, 500 // max_dim))
        h_px    = info.height * scale
        w_px    = info.width  * scale
        img     = cv2.resize(img, (w_px, h_px), interpolation=cv2.INTER_NEAREST)
        img     = np.ascontiguousarray(np.flipud(img))

        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution

        def w2p(wx, wy):
            px = int((wx - ox) / res * scale)
            py = h_px - int((wy - oy) / res * scale)
            return (max(0, min(w_px-1, px)), max(0, min(h_px-1, py)))

        if start is not None:
            px, py = w2p(*start)
            r = max(5, scale*2)
            cv2.circle(img, (px, py), r, (50, 220, 80), -1)

        if final is not None:
            px, py = w2p(*final)
            r = max(8, scale*3)
            cv2.drawMarker(img, (px, py), (50, 60, 255), cv2.MARKER_CROSS, r*2, max(1, scale))

        if robot is not None:
            rx, ry, yaw = robot
            px, py = w2p(rx, ry)
            al = max(8, scale*4)
            ex = int(px + al * math.cos(yaw))
            ey = int(py - al * math.sin(yaw))
            cv2.circle(img, (px, py), max(4, scale*2), (0, 165, 255), -1)
            cv2.arrowedLine(img, (px, py), (ex, ey), (0, 165, 255), max(1, scale), tipLength=0.45)

        _, buf = cv2.imencode('.png', img)
        return buf.tobytes()

    # ── stats JSON ─────────────────────────────────────────────────────────────

    def get_stats(self, launcher_mode: str) -> str:
        with self._lock:
            robot = ({'x': round(self._robot_xyz[0], 3),
                      'y': round(self._robot_xyz[1], 3)}
                     if self._robot_xyz else None)
            start = ({'x': round(self._start_xy[0], 3),
                      'y': round(self._start_xy[1], 3)}
                     if self._start_xy else None)
            final = ({'x': round(self._final_xy[0], 3),
                      'y': round(self._final_xy[1], 3)}
                     if self._final_xy else None)
            return json.dumps({
                'launcher_mode': launcher_mode,
                'status':   self._status,
                'prob':     round(self._prob, 3),
                'detected': self._detected,
                'elapsed':  self._elapsed,
                'robot':    robot,
                'start':    start,
                'final':    final,
            })


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PhoenixLauncher()
    pm   = _ProcManager()

    _Handler.node = node
    _Handler.pm   = pm
    node._ui_log  = pm._put   # lets the ROS node write to the web UI log

    server = _ThreadedServer(('0.0.0.0', 8081), _Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_raw_camera()
        pm.stop_all(save_rtt_map=False)
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
