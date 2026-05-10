#!/usr/bin/env python3
"""
Unified Robot Control — Web Server + Serial Bridge.

Runs on the Raspberry Pi. Hosts a web UI for controlling the robot
and capturing images from the ESP32-CAM, all over USB serial to the
Arduino R4 WiFi.

Usage:
    python3 robot_pi.py              # auto-detect port
    python3 robot_pi.py /dev/ttyACM0 # explicit port
"""

import serial
import serial.tools.list_ports
import struct
import sys
import os
import time
import datetime
import threading
import json
import io
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

try:
    from google import genai as _genai
    import PIL.Image
    _GEMINI_AVAILABLE = True
except ImportError:
    _GEMINI_AVAILABLE = False

BAUD_RATE = 921600
WEB_PORT = 8080

VALID_DIRS = {"F", "B", "L", "R", "FL", "FR", "BL", "BR", "S"}

GEMINI_API_KEY = os.environ.get("GEMINI_API_KEY", "")
GEMINI_MODEL   = "gemini-robotics-er-1.6-preview"
GEMINI_DIR_MAP = {
    "move_forward":  "F",
    "move_backward": "B",
    "turn_left":     "L",
    "turn_right":    "R",
    "stop":          "S",
}

def gemini_decide(jpeg_bytes: bytes, goal: str) -> str:
    """Send a JPEG frame to Gemini Robotics-ER and return a direction code."""
    if not _GEMINI_AVAILABLE:
        raise RuntimeError("google-genai not installed — run: pip install google-genai Pillow")
    if not GEMINI_API_KEY:
        raise RuntimeError("GEMINI_API_KEY environment variable not set")
    client = _genai.Client(api_key=GEMINI_API_KEY)
    img = PIL.Image.open(io.BytesIO(jpeg_bytes))
    resp = client.models.generate_content(
        model=GEMINI_MODEL,
        contents=[
            img,
            f"You control a wheeled robot car. Goal: {goal}. "
            "Reply with exactly one of these words and nothing else: "
            "move_forward, move_backward, turn_left, turn_right, stop.",
        ],
    )
    token = resp.text.strip().lower()
    return GEMINI_DIR_MAP.get(token, "S")


def find_arduino_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").lower()
        if "arduino" in desc or "ACM" in port.device or "usbmodem" in port.device:
            return port.device
    if len(ports) == 1:
        return ports[0].device
    return None


class RobotSerial:
    """Thread-safe serial interface to the Arduino."""

    def __init__(self, port_name):
        self.port_name = port_name
        self.ser = None
        self.lock = threading.Lock()
        self.speed = 200
        self.connected = False
        self.last_image_path = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.port_name, BAUD_RATE, timeout=2)
            print(f"[Serial] Connected to {self.port_name} at {BAUD_RATE} baud")
        except Exception as e:
            print(f"[Serial] Failed: {e}")
            return False

        print("[Serial] Resetting Arduino...")
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.dtr = True
        time.sleep(1)
        self.ser.reset_input_buffer()

        print("[Serial] Waiting for SYSTEM_READY (up to 120s for WiFi)...")
        timeout_start = time.time()
        while True:
            elapsed = time.time() - timeout_start
            if elapsed > 120:
                print("[Serial] SYSTEM_READY not received — proceeding anyway (camera may be unavailable)")
                break
            self.ser.timeout = 1
            raw = self.ser.readline()
            if raw:
                line = raw.decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"  [Arduino] {line}")
                    if "SYSTEM_READY" in line:
                        break
            elif elapsed > 5:
                # No data at all after 5s — Arduino likely didn't reset via DTR.
                # Send a newline to probe it; if still nothing after 120s we fall through.
                self.ser.write(b'\n')

        self.connected = True
        print("[Serial] Arduino ready!")
        return True

    def send_motor(self, direction, speed=None):
        if speed is None:
            speed = self.speed
        with self.lock:
            try:
                cmd = f"CMD:{direction} {speed}\n"
                self.ser.write(cmd.encode('ascii'))
                self.ser.flush()
                # Quick ack read
                deadline = time.time() + 0.3
                while time.time() < deadline:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline()
                        line = raw.decode('utf-8', errors='ignore').strip()
                        if "CMD_OK" in line:
                            return True
                        if "CMD_ERR" in line:
                            return False
                    time.sleep(0.01)
                return True
            except Exception as e:
                print(f"[Serial] Error: {e}")
                return False

    def capture_image(self):
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(b'S')
                self.ser.flush()

                buffer = bytearray()
                timeout_start = time.time()

                while True:
                    if time.time() - timeout_start > 30:
                        return None, "Timeout"

                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        chunk = self.ser.read(waiting)
                    else:
                        chunk = self.ser.read(1)
                        if not chunk:
                            continue

                    buffer.extend(chunk)
                    magic_idx = buffer.find(b'\xff\xaa\xbb\xcc')

                    if magic_idx == -1:
                        # Drain text lines
                        while b'\n' in buffer:
                            nl = buffer.index(b'\n')
                            line = buffer[:nl].decode('utf-8', errors='ignore').strip()
                            if line:
                                print(f"  [Camera] {line}")
                            del buffer[:nl+1]
                        continue

                    buffer = buffer[magic_idx:]

                    while len(buffer) < 8:
                        more = self.ser.read(8 - len(buffer))
                        if more:
                            buffer.extend(more)

                    img_length = struct.unpack('>I', buffer[4:8])[0]
                    print(f"  [Camera] Receiving {img_length} bytes...")

                    total_needed = 8 + img_length
                    while len(buffer) < total_needed:
                        remaining = total_needed - len(buffer)
                        more = self.ser.read(min(remaining, 4096))
                        if more:
                            buffer.extend(more)

                    img_data = buffer[8:8 + img_length]

                    if img_data[:2] == b'\xff\xd8':
                        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        filename = f"capture_{ts}.jpg"
                        with open(filename, 'wb') as f:
                            f.write(img_data)
                        self.last_image_path = os.path.abspath(filename)
                        print(f"  [Camera] Saved: {filename} ({img_length} bytes)")
                        return img_data, filename
                    else:
                        return None, "Not a JPEG"

            except Exception as e:
                return None, str(e)

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'CMD:S 0\n')
                self.ser.flush()
            except:
                pass
            self.ser.close()


# ── Global robot instance ──
robot = None


def get_html():
    """Return the control panel HTML."""
    return """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Robot Control</title>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg: #0a0e1a;
    --surface: rgba(255,255,255,0.05);
    --surface-hover: rgba(255,255,255,0.1);
    --border: rgba(255,255,255,0.08);
    --text: #e2e8f0;
    --text-dim: #64748b;
    --accent: #3b82f6;
    --accent-glow: rgba(59,130,246,0.3);
    --danger: #ef4444;
    --success: #22c55e;
    --orange: #f97316;
  }

  body {
    font-family: 'Inter', sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100dvh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 1rem;
    -webkit-user-select: none;
    user-select: none;
    overflow-x: hidden;
  }

  h1 {
    font-size: 1.4rem;
    font-weight: 700;
    letter-spacing: -0.02em;
    margin-bottom: 0.25rem;
    background: linear-gradient(135deg, #60a5fa, #a78bfa);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
  }

  .status {
    font-size: 0.75rem;
    color: var(--success);
    margin-bottom: 1rem;
  }

  /* ── D-Pad ── */
  .dpad {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    grid-template-rows: repeat(3, 1fr);
    gap: 6px;
    width: min(280px, 80vw);
    aspect-ratio: 1;
    margin-bottom: 1.2rem;
  }

  .dpad button {
    border: 1px solid var(--border);
    border-radius: 12px;
    background: var(--surface);
    color: var(--text);
    font-size: 1.1rem;
    font-weight: 600;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    transition: all 0.15s;
    -webkit-tap-highlight-color: transparent;
    touch-action: manipulation;
  }

  .dpad button:active, .dpad button.active {
    background: var(--accent);
    border-color: var(--accent);
    box-shadow: 0 0 20px var(--accent-glow);
    transform: scale(0.95);
  }

  .dpad button.stop-btn {
    background: rgba(239,68,68,0.15);
    border-color: rgba(239,68,68,0.3);
    color: var(--danger);
    font-size: 0.9rem;
  }

  .dpad button.stop-btn:active, .dpad button.stop-btn.active {
    background: var(--danger);
    color: white;
    box-shadow: 0 0 20px rgba(239,68,68,0.4);
  }

  .dpad .empty { visibility: hidden; }

  /* ── Controls Row ── */
  .controls {
    display: flex;
    gap: 0.75rem;
    align-items: center;
    width: min(280px, 80vw);
    margin-bottom: 1rem;
  }

  .controls label {
    font-size: 0.75rem;
    color: var(--text-dim);
    white-space: nowrap;
  }

  .controls input[type=range] {
    flex: 1;
    accent-color: var(--accent);
    height: 6px;
  }

  .speed-val {
    font-size: 0.85rem;
    font-weight: 600;
    min-width: 2ch;
    color: var(--accent);
  }

  /* ── Action Buttons ── */
  .actions {
    display: flex;
    gap: 0.5rem;
    width: min(280px, 80vw);
    margin-bottom: 1rem;
  }

  .actions button {
    flex: 1;
    padding: 0.7rem;
    border-radius: 10px;
    border: 1px solid var(--border);
    background: var(--surface);
    color: var(--text);
    font-family: inherit;
    font-size: 0.8rem;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.15s;
  }

  .actions button:hover { background: var(--surface-hover); }

  .actions .capture-btn {
    background: rgba(249,115,22,0.15);
    border-color: rgba(249,115,22,0.3);
    color: var(--orange);
  }

  .actions .capture-btn:hover {
    background: rgba(249,115,22,0.25);
  }

  .actions .capture-btn:disabled {
    opacity: 0.5;
    cursor: wait;
  }

  /* ── Image Preview ── */
  .preview {
    width: min(280px, 80vw);
    border-radius: 12px;
    overflow: hidden;
    border: 1px solid var(--border);
    background: var(--surface);
    display: none;
  }

  .preview img {
    width: 100%;
    display: block;
  }

  .preview .meta {
    padding: 0.5rem 0.75rem;
    font-size: 0.7rem;
    color: var(--text-dim);
  }

  /* ── Log ── */
  .log {
    width: min(280px, 80vw);
    margin-top: 0.75rem;
    padding: 0.5rem 0.75rem;
    border-radius: 10px;
    background: var(--surface);
    border: 1px solid var(--border);
    font-size: 0.65rem;
    font-family: 'SF Mono', 'Fira Code', monospace;
    color: var(--text-dim);
    max-height: 100px;
    overflow-y: auto;
    word-break: break-all;
  }
</style>
</head>
<body>

<h1>🤖 Robot Control</h1>
<div class="status" id="status">● Connected</div>

<div class="dpad">
  <button data-dir="FL">↖</button>
  <button data-dir="F">▲</button>
  <button data-dir="FR">↗</button>
  <button data-dir="L">◀</button>
  <button class="stop-btn" data-dir="S">STOP</button>
  <button data-dir="R">▶</button>
  <button data-dir="BL">↙</button>
  <button data-dir="B">▼</button>
  <button data-dir="BR">↘</button>
</div>

<div class="controls">
  <label>Speed</label>
  <input type="range" id="speed" min="0" max="255" value="200">
  <span class="speed-val" id="speedVal">200</span>
</div>

<div class="actions">
  <button class="capture-btn" id="captureBtn" onclick="capture()">📸 Capture</button>
  <button class="capture-btn" id="geminiBtn" onclick="geminiStep()">🤖 Gemini</button>
</div>

<div class="preview" id="preview">
  <img id="previewImg" src="" alt="Captured image">
  <div class="meta" id="previewMeta"></div>
</div>

<div class="log" id="log"></div>

<script>
const speedEl = document.getElementById('speed');
const speedVal = document.getElementById('speedVal');
const logEl = document.getElementById('log');
let driving = null;
let driveInterval = null;

speedEl.addEventListener('input', () => {
  speedVal.textContent = speedEl.value;
});

function log(msg) {
  const t = new Date().toLocaleTimeString();
  logEl.textContent = t + ' ' + msg + '\\n' + logEl.textContent;
  if (logEl.textContent.length > 2000) logEl.textContent = logEl.textContent.slice(0, 2000);
}

function sendCmd(dir) {
  const speed = speedEl.value;
  fetch('/api/motor?dir=' + dir + '&speed=' + speed)
    .then(r => r.json())
    .then(d => { if (!d.ok) log('ERR: ' + d.error); })
    .catch(e => log('ERR: ' + e));
}

function startDrive(dir) {
  if (driving === dir) return;
  stopDrive();
  driving = dir;
  log('Drive: ' + dir);
  sendCmd(dir);
  driveInterval = setInterval(() => sendCmd(dir), 200);
}

function stopDrive() {
  if (driving) {
    clearInterval(driveInterval);
    driveInterval = null;
    driving = null;
    sendCmd('S');
    log('Stop');
  }
}

// D-pad: hold to drive
document.querySelectorAll('.dpad button').forEach(btn => {
  const dir = btn.dataset.dir;

  function down(e) {
    e.preventDefault();
    btn.classList.add('active');
    if (dir === 'S') { stopDrive(); sendCmd('S'); log('Stop'); }
    else startDrive(dir);
  }
  function up(e) {
    e.preventDefault();
    btn.classList.remove('active');
    if (dir !== 'S') stopDrive();
  }

  btn.addEventListener('mousedown', down);
  btn.addEventListener('mouseup', up);
  btn.addEventListener('mouseleave', up);
  btn.addEventListener('touchstart', down, {passive: false});
  btn.addEventListener('touchend', up, {passive: false});
  btn.addEventListener('touchcancel', up, {passive: false});
});

// Keyboard
const keyMap = {
  'ArrowUp':'F','ArrowDown':'B','ArrowLeft':'L','ArrowRight':'R',
  'w':'F','s':'B','a':'L','d':'R',' ':'S'
};
const keysDown = new Set();
document.addEventListener('keydown', e => {
  const dir = keyMap[e.key];
  if (!dir || keysDown.has(e.key)) return;
  keysDown.add(e.key);
  e.preventDefault();
  if (dir === 'S') { stopDrive(); sendCmd('S'); }
  else startDrive(dir);
  const btn = document.querySelector(`[data-dir="${dir}"]`);
  if (btn) btn.classList.add('active');
});
document.addEventListener('keyup', e => {
  const dir = keyMap[e.key];
  if (!dir) return;
  keysDown.delete(e.key);
  e.preventDefault();
  if (dir !== 'S') stopDrive();
  const btn = document.querySelector(`[data-dir="${dir}"]`);
  if (btn) btn.classList.remove('active');
});

let audioCtx = null;
function initAudio() {
  if (!audioCtx) {
    audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  }
  if (audioCtx.state === 'suspended') audioCtx.resume();
}

function playBeep(dir) {
  if (!audioCtx) return;
  const osc = audioCtx.createOscillator();
  const gain = audioCtx.createGain();
  const freqs = { 'F': 660, 'B': 330, 'L': 550, 'R': 550, 'S': 220 };
  osc.frequency.value = freqs[dir] || 440;
  gain.gain.setValueAtTime(0.1, audioCtx.currentTime);
  gain.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + 0.3);
  osc.connect(gain);
  gain.connect(audioCtx.destination);
  osc.start();
  osc.stop(audioCtx.currentTime + 0.3);
}

let geminiGoal = 'avoid obstacles and move forward';
let geminiLoop = null;

function geminiStep() {
  initAudio();
  const btn = document.getElementById('geminiBtn');
  if (geminiLoop) {
    clearInterval(geminiLoop);
    geminiLoop = null;
    btn.textContent = '🤖 Gemini';
    log('Gemini loop stopped');
    sendCmd('S');
    return;
  }
  const goal = prompt('Goal for Gemini?', geminiGoal);
  if (!goal) return;
  geminiGoal = goal;
  btn.textContent = '⏹ Stop AI';
  log('Gemini loop started: ' + goal);

  function step() {
    fetch('/api/gemini?goal=' + encodeURIComponent(geminiGoal))
      .then(r => r.json())
      .then(d => {
        if (d.ok) {
          log('Gemini → ' + d.direction);
          playBeep(d.direction);
        }
        else { log('Gemini ERR: ' + d.error); clearInterval(geminiLoop); geminiLoop = null; btn.textContent = '🤖 Gemini'; }
      })
      .catch(e => { log('Gemini ERR: ' + e); clearInterval(geminiLoop); geminiLoop = null; btn.textContent = '🤖 Gemini'; });
  }
  step();
  geminiLoop = setInterval(step, 2500);
}

function capture() {
  const btn = document.getElementById('captureBtn');
  btn.disabled = true;
  btn.textContent = '⏳ Capturing...';
  log('Capturing image...');
  fetch('/api/capture')
    .then(r => r.json())
    .then(d => {
      btn.disabled = false;
      btn.textContent = '📸 Capture';
      if (d.ok) {
        log('Image saved: ' + d.filename);
        const preview = document.getElementById('preview');
        const img = document.getElementById('previewImg');
        const meta = document.getElementById('previewMeta');
        img.src = '/api/image?t=' + Date.now();
        meta.textContent = d.filename + ' — ' + d.size + ' bytes';
        preview.style.display = 'block';
      } else {
        log('Capture failed: ' + d.error);
      }
    })
    .catch(e => {
      btn.disabled = false;
      btn.textContent = '📸 Capture';
      log('Capture error: ' + e);
    });
}
</script>
</body>
</html>"""


class RobotHTTPHandler(BaseHTTPRequestHandler):
    """Handles web UI and API requests."""

    def log_message(self, format, *args):
        # Suppress default access logs
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/' or path == '/index.html':
            self._respond(200, 'text/html', get_html())

        elif path == '/api/motor':
            params = parse_qs(parsed.query)
            direction = params.get('dir', ['S'])[0].upper()
            speed = int(params.get('speed', ['200'])[0])

            if direction not in VALID_DIRS:
                self._json(400, {'ok': False, 'error': 'bad direction'})
                return

            robot.speed = speed
            ok = robot.send_motor(direction, speed)
            self._json(200, {'ok': ok})

        elif path == '/api/capture':
            img_data, result = robot.capture_image()
            if img_data:
                self._json(200, {
                    'ok': True,
                    'filename': result,
                    'size': len(img_data)
                })
            else:
                self._json(200, {'ok': False, 'error': result})

        elif path == '/api/image':
            if robot.last_image_path and os.path.exists(robot.last_image_path):
                with open(robot.last_image_path, 'rb') as f:
                    data = f.read()
                self._respond(200, 'image/jpeg', data, binary=True)
            else:
                self._respond(404, 'text/plain', 'No image')

        elif path == '/api/gemini':
            params = parse_qs(parsed.query)
            goal = params.get('goal', ['avoid obstacles and move forward'])[0]
            img_data, result = robot.capture_image()
            if not img_data:
                self._json(200, {'ok': False, 'error': result})
                return
            try:
                direction = gemini_decide(img_data, goal)
                robot.send_motor(direction)
                self._json(200, {'ok': True, 'direction': direction, 'goal': goal})
            except Exception as e:
                self._json(200, {'ok': False, 'error': str(e)})

        else:
            self._respond(404, 'text/plain', 'Not found')

    def _respond(self, code, content_type, body, binary=False):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        if not binary:
            body = body.encode('utf-8')
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, obj):
        self._respond(code, 'application/json', json.dumps(obj))


def main() -> int:
    global robot

    port = sys.argv[1] if len(sys.argv) > 1 else find_arduino_port()

    if not port:
        print("Could not find Arduino serial port.")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
        print("\nUsage: python3 robot_pi.py [/dev/ttyACM0]")
        return 1

    robot = RobotSerial(port)
    if not robot.connect():
        return 1

    # Start web server
    server = HTTPServer(('0.0.0.0', WEB_PORT), RobotHTTPHandler)
    print(f"\n{'='*50}")
    print(f"  Web UI: http://localhost:{WEB_PORT}")

    # Also show LAN IP for phone access
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        print(f"  Phone:  http://{ip}:{WEB_PORT}")
    except:
        pass

    print(f"{'='*50}\n")
    print("Press Ctrl+C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        robot.close()
        server.server_close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
