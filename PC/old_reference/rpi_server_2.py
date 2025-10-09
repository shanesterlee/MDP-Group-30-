#!/usr/bin/env python3
"""
rpi_server_bt_integrated.py
---------------------------
Bluetooth-integrated RPi Server that:
- Receives OBJECT/ROBOT data from Android via RFCOMM
- Maintains obstacle grid dynamically
- Sends full JSON to algorithm when 'y' is received
"""

from bluetooth import *
import threading
import json
import time
import serial
import base64
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
import logging
import signal
import sys
import requests
import cv2
from old_reference.bt_message_mapper import BTMessageMapper  # ✅ used for parsing

# =============================================================================
# Configuration
# =============================================================================
WLAN_HOST = '0.0.0.0'
WLAN_PORT = 5000
STM_PORT = '/dev/ttyACM0'
STM_BAUD = 115200
TASK_TIMEOUT = 300
API_SERVER = "http://127.0.0.1:5000"

# =============================================================================
# Logging
# =============================================================================
logging.basicConfig(level=logging.INFO,
                    format='[%(asctime)s] %(levelname)s: %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger("RPI_BT_SERVER")

# =============================================================================
# Global Robot State
# =============================================================================
class RobotState:
    def __init__(self):
        self.obstacles = {}   # object_id -> obstacle dict
        self.robot_pos = {'x': 0.0, 'y': 0.0, 'direction': 'N'}
        self.lock = threading.Lock()
        logger.info("[STATE] Initialized empty grid waiting for Bluetooth data.")

    def update_object(self, object_id, x, y, face):
        """Store obstacle in meters with matching schema."""
        with self.lock:
            self.obstacles[str(object_id)] = {
                'id': str(object_id),
                'x': round(x / 10.0, 2),
                'y': round(y / 10.0, 2),
                'face': face,
                'image_id': None
            }
            logger.info(f"[STATE] Updated OBJECT{object_id} → "
                        f"({self.obstacles[str(object_id)]['x']:.2f}, "
                        f"{self.obstacles[str(object_id)]['y']:.2f}) facing {face}")

    def update_robot(self, x, y, direction):
        with self.lock:
            self.robot_pos.update({
                'x': round(x / 10.0, 2),
                'y': round(y / 10.0, 2),
                'direction': direction
            })
            logger.info(f"[STATE] Updated ROBOT → {self.robot_pos}")

    def export_grid(self):
        """Return sorted list of obstacles for transmission."""
        with self.lock:
            grid = sorted(self.obstacles.values(), key=lambda o: int(o['id']))
            logger.info(f"[STATE] Final obstacle grid ready ({len(grid)} obstacles)")
            return grid

    def mark_ready(self):
        """Triggered when Android sends 'y' → send full grid to API."""
        grid = self.export_grid()
        payload = {'obstacles': grid, 'robot_pos': self.robot_pos}
        try:
            resp = requests.post(f"{API_SERVER}/start_task", json=payload, timeout=5)
            logger.info(f"[HTTP] Sent full grid to server ({len(grid)} obstacles).")
            logger.info(f"[HTTP] Response {resp.status_code}: {resp.text}")
        except Exception as e:
            logger.error(f"[HTTP] Failed to send to API: {e}")

    def get_state(self):
        with self.lock:
            return {'obstacles': list(self.obstacles.values()), 'robot_pos': dict(self.robot_pos)}

state = RobotState()

# =============================================================================
# Bluetooth Handler
# =============================================================================
running = True
def signal_handler(sig, frame):
    global running
    print("\n[RPi] Shutting down...")
    running = False
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def handle_bt_session(client_sock, client_info):
    print(f"[BT] Connected to {client_info}")
    stop_flag = [False]

    def receiver():
        while not stop_flag[0]:
            try:
                data = client_sock.recv(1024).decode("utf-8").strip()
                if not data:
                    print("[BT] Disconnected")
                    stop_flag[0] = True
                    break

                print(f"[Android → RPi] Raw: {data}")
                parsed = BTMessageMapper.parse_android_message(data)
                print(f"[Android → RPi] Parsed: {parsed}")

                if not parsed:
                    continue

                if parsed['type'] == 'object_pos':
                    state.update_object(parsed['object_id'], parsed['x'], parsed['y'], parsed['direction'])
                elif parsed['type'] == 'robot_pos':
                    state.update_robot(parsed['x'], parsed['y'], parsed['direction'])
                elif parsed['type'] == 'ready_signal':
                    state.mark_ready()

            except Exception as e:
                logger.error(f"[BT Receiver] {e}")
                stop_flag[0] = True
                break

    threading.Thread(target=receiver, daemon=True).start()
    try:
        while not stop_flag[0]:
            time.sleep(0.2)
    except KeyboardInterrupt:
        stop_flag[0] = True
    client_sock.close()
    print("[BT] Session closed")

def start_bt_server():
    print("[BT] Starting RFCOMM server...")
    server_sock = BluetoothSocket(RFCOMM)
    server_sock.bind(("", 1))
    server_sock.listen(1)
    print("[BT] Waiting for Android connection (SPP channel 1)...")
    while running:
        try:
            client_sock, client_info = server_sock.accept()
            handle_bt_session(client_sock, client_info)
        except OSError:
            break
    server_sock.close()

# =============================================================================
# HTTP Server (for PC/Algorithm)
# =============================================================================
class RPIRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def _send_json(self, data, code=200):
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_GET(self):
        if self.path == "/status":
            self._send_json(state.get_state())
        else:
            self._send_json({"error": "unknown endpoint"}, 404)

def start_http_server():
    server = HTTPServer((WLAN_HOST, WLAN_PORT), RPIRequestHandler)
    logger.info(f"[HTTP] Listening on {WLAN_HOST}:{WLAN_PORT}")
    server.serve_forever()

# =============================================================================
# Main Entry
# =============================================================================
def main():
    logger.info("=== RPI MDP SERVER WITH BLUETOOTH GRID ===")
    threading.Thread(target=start_http_server, daemon=True).start()
    start_bt_server()

if __name__ == "__main__":
    main()
