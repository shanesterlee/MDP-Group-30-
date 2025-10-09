#!/usr/bin/env python3
"""
rpi_server_test.py - Test version with preset obstacles

Modified version of rpi_server.py that loads preset obstacles
instead of waiting for Android Bluetooth input.
This is to test if the photos form the tiled version. If yes just have to add android communication to this.

Usage:
    python rpi_server_test.py
"""

import socket
import json
import threading
import time
import serial
import io
import base64
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import logging

import cv2

# Check if an OpenCV camera can be opened
cap_test = cv2.VideoCapture(0)
if cap_test.isOpened():
    CAMERA_AVAILABLE = True
    cap_test.release()
    print("[INFO] OpenCV camera detected and available.")
else:
    CAMERA_AVAILABLE = False
    print("[WARN] No OpenCV camera detected using mock camera.")


# =============================================================================
# Configuration
# =============================================================================
WLAN_HOST = '0.0.0.0'
WLAN_PORT = 5000
STM_PORT = '/dev/ttyACM0'
STM_BAUD = 115200
TASK_TIMEOUT = 300

# =============================================================================
# PRESET OBSTACLES FOR TESTING
# =============================================================================
# Define your test grid here - modify as needed
PRESET_OBSTACLES = [
    {'id': '1', 'x': 0.30, 'y': 0.30, 'face': 'N', 'image_id': None},
    {'id': '2', 'x': 0.60, 'y': 0.30, 'face': 'S', 'image_id': None},
    {'id': '3', 'x': 0.90, 'y': 0.30, 'face': 'E', 'image_id': None},
    {'id': '4', 'x': 1.20, 'y': 0.60, 'face': 'W', 'image_id': None},
    {'id': '5', 'x': 1.50, 'y': 0.90, 'face': 'N', 'image_id': None},
]

# Alternative: Simple 3-obstacle test
# PRESET_OBSTACLES = [
#     {'id': 'A', 'x': 0.40, 'y': 0.40, 'face': 'N', 'image_id': None},
#     {'id': 'B', 'x': 1.00, 'y': 0.60, 'face': 'E', 'image_id': None},
#     {'id': 'C', 'x': 1.40, 'y': 1.20, 'face': 'S', 'image_id': None},
# ]

# =============================================================================
# Logging Setup
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# =============================================================================
# Global State
# =============================================================================
class RobotState:
    def __init__(self):
        # Initialize with preset obstacles
        self.obstacles = PRESET_OBSTACLES.copy()
        self.robot_pos = {'x': 0.125, 'y': 0.1, 'heading': 90}
        self.task_start_time = None
        self.task_active = False
        self.last_stm_response = None
        self.identified_count = 0
        self.lock = threading.Lock()
        
        logger.info(f"Initialized with {len(self.obstacles)} preset obstacles")
        
    def update_robot_pos(self, heading, x, y):
        with self.lock:
            self.robot_pos['heading'] = heading
            self.robot_pos['x'] = x / 100.0
            self.robot_pos['y'] = y / 100.0
            logger.info(f"Robot pos updated: {self.robot_pos}")
    
    def update_obstacle_image(self, obstacle_id, image_id):
        with self.lock:
            for obs in self.obstacles:
                if obs['id'] == obstacle_id:
                    obs['image_id'] = image_id
                    self.identified_count += 1
                    logger.info(f"Obstacle {obstacle_id} identified as {image_id}")
                    return True
        return False
    
    def get_state_dict(self):
        with self.lock:
            return {
                'obstacles': self.obstacles.copy(),
                'robot_pos': self.robot_pos.copy(),
                'task_active': self.task_active,
                'identified_count': self.identified_count,
                'elapsed_time': time.time() - self.task_start_time if self.task_start_time else 0
            }

state = RobotState()

# =============================================================================
# STM Serial Communication
# =============================================================================
class STMComm:
    def __init__(self, port=STM_PORT, baud=STM_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.response_buffer = ""
        self.response_ready = threading.Event()
        self.lock = threading.Lock()
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            logger.info(f"Connected to STM on {self.port} @ {self.baud}")
            threading.Thread(target=self._listen_responses, daemon=True).start()
            return True
        except Exception as e:
            logger.error(f"Failed to connect to STM: {e}")
            return False
    
    def _listen_responses(self):
        while self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    if line:
                        logger.info(f"STM << {line}")
                        self._process_response(line)
            except Exception as e:
                logger.error(f"Error reading STM: {e}")
            time.sleep(0.01)
    
    def _process_response(self, line):
        try:
            parts = line.split(',')
            if len(parts) == 3:
                heading = int(parts[0].strip())
                x = int(parts[1].strip())
                y = int(parts[2].strip())
                state.update_robot_pos(heading, x, y)
                with self.lock:
                    self.response_buffer = line
                    self.response_ready.set()
        except Exception as e:
            logger.warning(f"Failed to parse STM response: {line} - {e}")
    
    def send_command(self, cmd, timeout=30):
        if not self.ser or not self.ser.is_open:
            logger.error("STM not connected")
            return None
        
        try:
            self.response_ready.clear()
            msg = f"{cmd}\n"
            self.ser.write(msg.encode('ascii'))
            logger.info(f"STM >> {cmd}")
            
            if self.response_ready.wait(timeout):
                with self.lock:
                    return self.response_buffer
            else:
                logger.error(f"STM command timeout: {cmd}")
                return None
        except Exception as e:
            logger.error(f"Error sending STM command: {e}")
            return None
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("STM connection closed")

stm = STMComm()

# =============================================================================
# Camera Interface (Fixed - using working camera.py approach)
# =============================================================================
class CameraInterface:
    def __init__(self):
        """Initialize camera - just verify it exists, don't keep it open"""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            logger.error("[ERROR] Could not open camera. Check connection or enable via raspi-config.")
            self.available = False
        else:
            logger.info("[INFO] OpenCV camera initialized successfully.")
            self.available = True
        cap.release()

    def capture_image(self):
        """Capture one fresh frame and return as base64 JPEG."""
        if not self.available:
            logger.warning("[WARN] Camera not available, returning mock image.")
            mock_jpeg = base64.b64encode(b'\xff\xd8\xff\xe0\x00\x10JFIF').decode('utf-8')
            return mock_jpeg
            
        try:
            # Open camera fresh for each capture (like camera.py does)
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                logger.error("[ERROR] Could not open camera for capture.")
                return None

            # Allow camera to warm up and auto-expose
            time.sleep(0.3)

            # Flush buffer by reading a few frames
            for _ in range(3):
                cap.read()
            
            # Capture the actual frame
            success, frame = cap.read()
            
            # Release immediately
            cap.release()

            if not success:
                logger.warning("[WARN] Failed to read new frame.")
                return None

            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                logger.error("[ERROR] JPEG encoding failed.")
                return None

            # Convert to base64
            img_b64 = base64.b64encode(buffer).decode('utf-8')
            logger.info(f"[INFO] Captured OpenCV frame ({len(buffer)} bytes).")
            return img_b64

        except Exception as e:
            logger.error(f"[ERROR] OpenCV capture failed: {e}")
            return None

    def close(self):
        """Cleanup - nothing to do since we don't keep camera open"""
        logger.info("Camera interface closed.")

# Create global camera instance
camera = CameraInterface()

# =============================================================================
# HTTP Server for PC Communication
# =============================================================================
class RPIRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass
    
    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))
    
    def do_GET(self):
        parsed = urlparse(self.path)
        
        if parsed.path == '/status':
            self._send_json(state.get_state_dict())
        
        elif parsed.path == '/obstacles':
            self._send_json({'obstacles': state.obstacles})
        
        else:
            self._send_json({'error': 'Unknown endpoint'}, 404)
    
    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8')
        
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            self._send_json({'error': 'Invalid JSON'}, 400)
            return
        
        parsed = urlparse(self.path)
        
        if parsed.path == '/start_task':
            state.task_start_time = time.time()
            state.task_active = True
            state.identified_count = 0
            logger.info("Task started")
            self._send_json({'status': 'started'})
        
        elif parsed.path == '/stm_command':
            cmd = data.get('command', '')
            if not cmd:
                self._send_json({'error': 'No command provided'}, 400)
                return
            
            response = stm.send_command(cmd)
            if response:
                self._send_json({'status': 'ok', 'response': response})
            else:
                self._send_json({'status': 'timeout'}, 408)
        
        elif parsed.path == '/capture':
            obstacle_id = data.get('obstacle_id')
            img_b64 = camera.capture_image()
            
            if img_b64:
                self._send_json({
                    'status': 'ok',
                    'obstacle_id': obstacle_id,
                    'image': img_b64
                })
            else:
                self._send_json({'status': 'error', 'message': 'Capture failed'}, 500)
        
        elif parsed.path == '/update_image':
            obstacle_id = data.get('obstacle_id')
            image_id = data.get('image_id')
            
            if state.update_obstacle_image(obstacle_id, image_id):
                self._send_json({'status': 'ok'})
            else:
                self._send_json({'error': 'Obstacle not found'}, 404)
        
        elif parsed.path == '/stop_task':
            state.task_active = False
            elapsed = time.time() - state.task_start_time if state.task_start_time else 0
            logger.info(f"Task stopped. Elapsed: {elapsed:.1f}s, Identified: {state.identified_count}")
            self._send_json({'status': 'stopped', 'elapsed': elapsed})
        
        else:
            self._send_json({'error': 'Unknown endpoint'}, 404)

def start_http_server():
    server = HTTPServer((WLAN_HOST, WLAN_PORT), RPIRequestHandler)
    logger.info(f"HTTP server listening on {WLAN_HOST}:{WLAN_PORT}")
    
    # Print obstacle info at startup
    logger.info("\n" + "="*70)
    logger.info("PRESET OBSTACLES LOADED:")
    for obs in PRESET_OBSTACLES:
        logger.info(f"  {obs['id']}: ({obs['x']:.2f}m, {obs['y']:.2f}m) facing {obs['face']}")
    logger.info("="*70 + "\n")
    
    server.serve_forever()

# =============================================================================
# Task Monitor
# =============================================================================
def task_monitor():
    while True:
        if state.task_active and state.task_start_time:
            elapsed = time.time() - state.task_start_time
            if elapsed > TASK_TIMEOUT:
                logger.warning(f"Task timeout ({TASK_TIMEOUT}s exceeded)")
                state.task_active = False
                stm.send_command('s')
        time.sleep(1)

# =============================================================================
# Main
# =============================================================================
def main():
    logger.info("=== RPI MDP Server (TEST MODE) ===")
    logger.info("Using PRESET obstacles - no Android Bluetooth required")
    
    # Connect to STM
    if not stm.connect():
        logger.error("Failed to connect to STM, exiting")
        return
    
    # Start task monitor
    threading.Thread(target=task_monitor, daemon=True).start()
    
    # Start HTTP server (blocking)
    try:
        start_http_server()
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
    finally:
        stm.close()
        camera.close()
        logger.info("Shutdown complete")

if __name__ == '__main__':
    main()