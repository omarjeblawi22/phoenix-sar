#!/usr/bin/env python3
"""
phoenix_dashboard.py — PHOENIX SAR Web Dashboard
=================================================

Serves a live web dashboard at http://<pi-ip>:8080

No external dependencies — uses only Python standard library + cv2/numpy
(both already installed for camera_detector_node).

Panels:
  LEFT       — Live SLAM map (robot arrow, start marker, target marker)
  TOP-RIGHT  — Camera feed with detection probability overlay
  BOT-RIGHT  — Mission status, elapsed time, position, probability
"""

import http.server
import json
import math
import socketserver
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

import tf2_ros


# ---------------------------------------------------------------------------
# HTML (embedded — no static files needed)
# ---------------------------------------------------------------------------

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
header{background:#161b22;border-bottom:1px solid #30363d;padding:8px 18px;
       display:flex;align-items:center;gap:14px;flex-shrink:0;min-height:46px}
header h1{font-size:1rem;color:#58a6ff;letter-spacing:3px;white-space:nowrap}
.badge{padding:3px 10px;border-radius:10px;font-size:.7rem;font-weight:bold;letter-spacing:1px;white-space:nowrap}
.bw{background:#21262d;color:#8b949e;border:1px solid #30363d}
.be{background:#0d419d;color:#79c0ff;border:1px solid #1f6feb}
.bs{background:#4d2d00;color:#e3b341;border:1px solid #9e6a03}
.bt{background:#033a16;color:#3fb950;border:1px solid #238636}
.hstat{font-size:.75rem;color:#8b949e;white-space:nowrap}
.hstat span{color:#c9d1d9}
.prob-wrap{margin-left:auto;display:flex;align-items:center;gap:8px;font-size:.75rem}
.pb-bg{width:110px;height:7px;background:#21262d;border-radius:4px;overflow:hidden}
.pb-fill{height:100%;background:#238636;border-radius:4px;transition:width .3s}
main{flex:1;display:grid;grid-template-columns:1fr 350px;gap:1px;background:#30363d;overflow:hidden;min-height:0}
.right-col{display:grid;grid-template-rows:1fr 290px;gap:1px;background:#30363d;min-height:0}
.panel{background:#0d1117;display:flex;flex-direction:column;overflow:hidden;min-height:0}
.ph{background:#161b22;border-bottom:1px solid #30363d;padding:5px 12px;
    font-size:.65rem;letter-spacing:2px;color:#58a6ff;font-weight:bold;flex-shrink:0}
.pc{flex:1;overflow:hidden;display:flex;align-items:center;justify-content:center;min-height:0}
#map-img{max-width:100%;max-height:100%;image-rendering:pixelated;image-rendering:crisp-edges}
#cam-img{max-width:100%;max-height:100%;object-fit:contain}
.stats-body{flex:1;overflow-y:auto;padding:8px 10px;display:flex;flex-direction:column;gap:6px}
.stat-row{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.sc{background:#161b22;border:1px solid #30363d;border-radius:5px;padding:7px 9px}
.sl{color:#8b949e;font-size:.6rem;letter-spacing:1px;text-transform:uppercase;margin-bottom:2px}
.sv{color:#c9d1d9;font-size:.8rem;font-weight:bold}
.sv.g{color:#3fb950}.sv.y{color:#e3b341}.sv.r{color:#f85149}
.prob-track{margin:0 10px 6px;height:3px;background:#21262d;border-radius:2px;overflow:hidden}
.prob-track-fill{height:100%;background:linear-gradient(90deg,#1f6feb,#3fb950);transition:width .4s}
</style>
</head>
<body>
<header>
  <h1>PHOENIX SAR</h1>
  <span id="badge" class="badge bw">WAITING</span>
  <div class="hstat">TIME&nbsp;<span id="h-time">--</span></div>
  <div class="hstat">POS&nbsp;<span id="h-pos">--</span></div>
  <div class="prob-wrap">
    <span id="h-pct">0%</span>
    <div class="pb-bg"><div id="h-pb" class="pb-fill" style="width:0%"></div></div>
  </div>
</header>
<main>
  <div class="panel">
    <div class="ph">SLAM MAP</div>
    <div class="pc" style="background:#0a0c0f;padding:2px">
      <img id="map-img" src="/map.png" alt="">
    </div>
  </div>
  <div class="right-col">
    <div class="panel">
      <div class="ph">CAMERA FEED</div>
      <div class="pc" style="background:#000;padding:0">
        <img id="cam-img" src="/camera" alt="">
      </div>
    </div>
    <div class="panel">
      <div class="ph">MISSION</div>
      <div class="prob-track"><div id="prob-fill" class="prob-track-fill" style="width:0%"></div></div>
      <div class="stats-body">
        <div class="stat-row">
          <div class="sc"><div class="sl">Status</div><div id="s-status" class="sv">WAITING</div></div>
          <div class="sc"><div class="sl">Elapsed</div><div id="s-elapsed" class="sv">--</div></div>
        </div>
        <div class="stat-row">
          <div class="sc"><div class="sl">Probability</div><div id="s-prob" class="sv">--</div></div>
          <div class="sc"><div class="sl">Target</div><div id="s-target" class="sv">NOT FOUND</div></div>
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
<script>
const BC={WAITING:'bw',EXPLORING:'be',SPINNING:'bs',TARGET_FOUND:'bt',MISSION_COMPLETE:'bt'};
function fp(p){return p?'('+p.x.toFixed(2)+', '+p.y.toFixed(2)+')':'--'}
function ft(s){if(s===null||s===undefined)return'--';var m=Math.floor(s/60),r=s%60;return m?m+'m '+r+'s':s+'s'}
async function poll(){
  try{
    var d=await fetch('/stats').then(r=>r.json());
    var pct=Math.round(d.prob*100);
    var cls=BC[d.status]||'bw';
    var b=document.getElementById('badge');
    b.textContent=d.status;b.className='badge '+cls;
    document.getElementById('h-time').textContent=ft(d.elapsed);
    document.getElementById('h-pos').textContent=fp(d.robot);
    document.getElementById('h-pct').textContent=pct+'%';
    document.getElementById('h-pb').style.width=pct+'%';
    var ss=document.getElementById('s-status');
    ss.textContent=d.status;
    ss.className='sv '+(d.status==='TARGET_FOUND'||d.status==='MISSION_COMPLETE'?'g':d.status==='SPINNING'?'y':'');
    document.getElementById('s-elapsed').textContent=ft(d.elapsed);
    var sp=document.getElementById('s-prob');
    sp.textContent=pct+'%';sp.className='sv '+(pct>70?'g':pct>40?'y':'');
    var st=document.getElementById('s-target');
    st.textContent=d.detected?'FOUND ★':'SCANNING...';
    st.className='sv '+(d.detected?'g':'');
    document.getElementById('s-robot').textContent=fp(d.robot);
    document.getElementById('s-start').textContent=fp(d.start);
    document.getElementById('s-final').textContent=d.final?fp(d.final)+(d.detected?' — TARGET':''):'--';
    document.getElementById('prob-fill').style.width=pct+'%';
    document.getElementById('map-img').src='/map.png?t='+Date.now();
  }catch(e){}
}
setInterval(poll,1000);poll();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# HTTP request handler (no Flask — pure stdlib)
# ---------------------------------------------------------------------------

class _Handler(http.server.BaseHTTPRequestHandler):
    """Route GET requests to the dashboard node's generators."""

    node: 'PhoenixDashboard' = None   # set before server starts

    def do_GET(self):
        path = self.path.split('?')[0]
        try:
            if path == '/':
                self._send_bytes(_HTML.encode('utf-8'), 'text/html; charset=utf-8')
            elif path == '/map.png':
                self._send_bytes(self.node.generate_map_png(), 'image/png', cache=False)
            elif path == '/stats':
                self._send_bytes(self.node.get_stats().encode(), 'application/json', cache=False)
            elif path == '/camera':
                self._stream_mjpeg()
            else:
                self.send_error(404)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _send_bytes(self, data: bytes, mime: str, cache: bool = True):
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        if not cache:
            self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _stream_mjpeg(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()

        offline = PhoenixDashboard._offline_jpg()
        prev = None
        while True:
            try:
                with self.node._lock:
                    jpg = self.node._frame_jpg
                if jpg is None:
                    jpg = offline
                if jpg is not prev:
                    prev = jpg
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                        + jpg + b'\r\n')
                    self.wfile.flush()
                else:
                    time.sleep(0.04)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    def log_message(self, *_):
        pass   # silence access logs in the ROS terminal


class _ThreadedServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True


# ---------------------------------------------------------------------------
# Dashboard ROS 2 node
# ---------------------------------------------------------------------------

class PhoenixDashboard(Node):

    def __init__(self):
        super().__init__('phoenix_dashboard')

        self._lock       = threading.Lock()
        self._map_msg    = None
        self._status     = 'WAITING'
        self._prob       = 0.0
        self._detected   = False
        self._frame_jpg  = None
        self._robot_xyz  = None   # (x, y, yaw)
        self._start_xy   = None   # (x, y)
        self._final_xy   = None   # (x, y)
        self._elapsed    = 0
        self._explore_t0 = None

        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid,   '/map',                    self._on_map,      map_qos)
        self.create_subscription(String,          '/phoenix/status',          self._on_status,   10)
        self.create_subscription(Float32,         '/phoenix/detection_prob',  self._on_prob,     10)
        self.create_subscription(Bool,            '/phoenix/target_detected', self._on_detected, 10)
        self.create_subscription(CompressedImage, '/phoenix/camera_frame',    self._on_frame,    1)
        self.create_subscription(PoseStamped,     '/phoenix/start_pose',      self._on_start,    10)
        self.create_subscription(PoseStamped,     '/phoenix/final_pose',      self._on_final,    10)

        self.create_timer(0.5, self._update_tf)

        self.get_logger().info(
            'Dashboard ready — open http://172.21.164.200:8080 in your browser')

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _on_map(self, msg):
        with self._lock:
            self._map_msg = msg

    def _on_status(self, msg):
        with self._lock:
            self._status = msg.data
            if msg.data == 'EXPLORING' and self._explore_t0 is None:
                self._explore_t0 = time.time()

    def _on_prob(self, msg):
        with self._lock:
            self._prob = float(msg.data)

    def _on_detected(self, msg):
        with self._lock:
            self._detected = bool(msg.data)

    def _on_frame(self, msg):
        arr   = np.frombuffer(bytes(msg.data), np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            prob, detected = self._prob, self._detected
        self._draw_overlay(frame, prob, detected)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if ok:
            with self._lock:
                self._frame_jpg = buf.tobytes()

    def _on_start(self, msg):
        with self._lock:
            self._start_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_final(self, msg):
        with self._lock:
            self._final_xy = (msg.pose.position.x, msg.pose.position.y)

    def _update_tf(self):
        try:
            tf  = self.tf_buf.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.1))
            t   = tf.transform.translation
            r   = tf.transform.rotation
            yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                             1.0 - 2.0 * (r.y * r.y + r.z * r.z))
            with self._lock:
                self._robot_xyz = (t.x, t.y, yaw)
                if self._explore_t0 is not None:
                    self._elapsed = int(time.time() - self._explore_t0)
        except Exception:
            pass

    # ── Camera overlay ────────────────────────────────────────────────────

    @staticmethod
    def _draw_overlay(frame, prob: float, detected: bool):
        h, w = frame.shape[:2]
        bar_col = (30, 200, 80) if detected else (30, 30, 30)
        txt_col = (50, 255, 100) if detected else (180, 180, 180)
        cv2.rectangle(frame, (0, 0), (w, 30), bar_col, -1)
        label = (f'TARGET FOUND  {prob*100:.0f}%' if detected
                 else f'Scanning...  {prob*100:.0f}%')
        cv2.putText(frame, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, txt_col, 2, cv2.LINE_AA)
        cv2.rectangle(frame, (0, h - 6), (w, h), (40, 40, 40), -1)
        fill_w = int(w * max(0.0, min(1.0, prob)))
        cv2.rectangle(frame, (0, h - 6), (fill_w, h),
                      (50, 200, 80) if detected else (50, 140, 220), -1)
        if detected:
            cv2.rectangle(frame, (2, 2), (w - 2, h - 2), (50, 255, 80), 3)

    @staticmethod
    def _offline_jpg() -> bytes:
        img = np.full((240, 320, 3), 18, dtype=np.uint8)
        cv2.putText(img, 'CAMERA OFFLINE', (40, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (55, 55, 55), 1, cv2.LINE_AA)
        _, buf = cv2.imencode('.jpg', img)
        return buf.tobytes()

    # ── Map rendering ─────────────────────────────────────────────────────

    def generate_map_png(self) -> bytes:
        with self._lock:
            map_msg  = self._map_msg
            robot    = self._robot_xyz
            start    = self._start_xy
            final    = self._final_xy

        if map_msg is None:
            img = np.full((200, 300, 3), 13, dtype=np.uint8)
            cv2.putText(img, 'Waiting for SLAM map...', (14, 106),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (65, 65, 65), 1, cv2.LINE_AA)
            _, buf = cv2.imencode('.png', img)
            return buf.tobytes()

        info = map_msg.info
        grid = np.array(map_msg.data, dtype=np.int8).reshape(
            info.height, info.width)

        img = np.full((info.height, info.width, 3), 55, dtype=np.uint8)
        img[(grid >= 0) & (grid <= 50)] = [220, 220, 220]
        img[grid > 50]                  = [22,  22,  22]

        max_dim = max(info.width, info.height, 1)
        scale   = max(1, min(8, 500 // max_dim))
        h_px    = info.height * scale
        w_px    = info.width  * scale
        img     = cv2.resize(img, (w_px, h_px), interpolation=cv2.INTER_NEAREST)
        img     = np.ascontiguousarray(np.flipud(img))

        ox, oy, res = (info.origin.position.x,
                       info.origin.position.y,
                       info.resolution)

        def w2p(wx, wy):
            px = int((wx - ox) / res * scale)
            py = h_px - int((wy - oy) / res * scale)
            return (max(0, min(w_px - 1, px)),
                    max(0, min(h_px - 1, py)))

        if start is not None:
            px, py = w2p(*start)
            r = max(5, scale * 2)
            cv2.circle(img, (px, py), r, (50, 220, 80), -1)
            cv2.putText(img, 'S', (px + r + 2, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.35, scale * 0.12), (50, 220, 80), 1, cv2.LINE_AA)

        if final is not None:
            px, py = w2p(*final)
            r = max(8, scale * 3)
            cv2.drawMarker(img, (px, py), (50, 60, 255),
                           cv2.MARKER_CROSS, r * 2, max(1, scale))
            cv2.putText(img, 'T', (px + r + 2, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.35, scale * 0.12), (50, 60, 255), 1, cv2.LINE_AA)

        if robot is not None:
            rx, ry, yaw = robot
            px, py  = w2p(rx, ry)
            arr_len = max(8, scale * 4)
            ex = int(px + arr_len * math.cos(yaw))
            ey = int(py - arr_len * math.sin(yaw))
            cv2.circle(img, (px, py), max(4, scale * 2), (0, 165, 255), -1)
            cv2.arrowedLine(img, (px, py), (ex, ey), (0, 165, 255),
                            max(1, scale), tipLength=0.45)

        _, buf = cv2.imencode('.png', img)
        return buf.tobytes()

    # ── Stats JSON ────────────────────────────────────────────────────────

    def get_stats(self) -> str:
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
                'status':   self._status,
                'prob':     round(self._prob, 3),
                'detected': self._detected,
                'elapsed':  self._elapsed,
                'robot':    robot,
                'start':    start,
                'final':    final,
            })


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PhoenixDashboard()

    _Handler.node = node
    server = _ThreadedServer(('0.0.0.0', 8080), _Handler)
    srv_thread = threading.Thread(target=server.serve_forever, daemon=True)
    srv_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
