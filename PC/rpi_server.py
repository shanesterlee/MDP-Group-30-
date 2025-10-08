#!/usr/bin/env python3
"""
rpi_server.py - Raspberry Pi coordinator for MDP robot navigation system

Responsibilities:
- Receive obstacle positions/orientations from Android via Bluetooth
- Host WLAN server for PC to fetch obstacle data
- Relay STM commands from PC to STM via serial
- Capture images and send to PC for recognition
- Track robot position and map state
- Update Android with progress

Architecture:
  Android <--BT--> RPI <--WLAN--> PC (Path Planner)
                   |
                   +--Serial--> STM (Motor Control)
                   |
                   +--Camera--> Image Capture
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

try:
    from picamera2 import Picamera2
    CAMERA_AVAILABLE = True
except ImportError:
    print("[WARN] picamera2 not available, using mock camera")
    CAMERA_AVAILABLE = False

try:
    import bluetooth
    BLUETOOTH_AVAILABLE = True
except ImportError:
    print("[WARN] bluetooth not available, using mock bluetooth")
    BLUETOOTH_AVAILABLE = False

# =============================================================================
# Configuration
# =============================================================================
WLAN_HOST = '0.0.0.0'  # Listen on all interfaces
WLAN_PORT = 5000
STM_PORT = '/dev/ttyUSB0'  # or '/dev/serial0' for GPIO UART
STM_BAUD = 115200
BLUETOOTH_NAME = "RPI_MDP_SERVER"
TASK_TIMEOUT = 300  # 5 minutes in seconds

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
        self.obstacles = []  # List of {id, x, y, face, image_id}
        self.robot_pos = {'x': 0.125, 'y': 0.1, 'heading': 90}  # Start pos
        self.task_start_time = None
        self.task_active = False
        self.last_stm_response = None
        self.identified_count = 0
        self.lock = threading.Lock()
        
    def update_robot_pos(self, heading, x, y):
        """Update robot position from STM feedback"""
        with self.lock:
            self.robot_pos['heading'] = heading
            self.robot_pos['x'] = x / 100.0  # STM sends cm, we use meters
            self.robot_pos['y'] = y / 100.0
            logger.info(f"Robot pos updated: {self.robot_pos}")
    
    def update_obstacle_image(self, obstacle_id, image_id):
        """Update identified image for an obstacle"""
        with self.lock:
            for obs in self.obstacles:
                if obs['id'] == obstacle_id:
                    obs['image_id'] = image_id
                    self.identified_count += 1
                    logger.info(f"Obstacle {obstacle_id} identified as {image_id}")
                    return True
        return False
    
    def get_state_dict(self):
        """Thread-safe state snapshot"""
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
        """Open serial connection to STM"""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)  # Allow STM to initialize
            logger.info(f"Connected to STM on {self.port} @ {self.baud}")
            # Start response listener thread
            threading.Thread(target=self._listen_responses, daemon=True).start()
            return True
        except Exception as e:
            logger.error(f"Failed to connect to STM: {e}")
            return False
    
    def _listen_responses(self):
        """Background thread to listen for STM responses"""
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
        """Parse STM response: heading, x, y"""
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
        """
        Send command to STM and wait for position response
        Commands: w<dist>, x<dist>, a<angle>, d<angle>, z<angle>, c<angle>, s, t<servo>
        """
        if not self.ser or not self.ser.is_open:
            logger.error("STM not connected")
            return None
        
        try:
            self.response_ready.clear()
            msg = f"{cmd}\n"
            self.ser.write(msg.encode('ascii'))
            logger.info(f"STM >> {cmd}")
            
            # Wait for response
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
# Camera Interface
# =============================================================================
class CameraInterface:
    def __init__(self):
        self.camera = None
        if CAMERA_AVAILABLE:
            try:
                self.camera = Picamera2()
                config = self.camera.create_still_configuration()
                self.camera.configure(config)
                self.camera.start()
                logger.info("Camera initialized")
            except Exception as e:
                logger.error(f"Camera init failed: {e}")
                self.camera = None
    
    def capture_image(self):
        """Capture image and return as base64 JPEG"""
        if self.camera:
            try:
                # Capture to buffer
                stream = io.BytesIO()
                self.camera.capture_file(stream, format='jpeg')
                stream.seek(0)
                img_data = stream.read()
                # Encode as base64
                img_b64 = base64.b64encode(img_data).decode('utf-8')
                logger.info(f"Captured image: {len(img_data)} bytes")
                return img_b64
            except Exception as e:
                logger.error(f"Image capture failed: {e}")
                return None
        else:
            # Mock image (1x1 red pixel JPEG)
            logger.warning("Using mock camera")
            mock_jpeg = base64.b64encode(b'\xff\xd8\xff\xe0\x00\x10JFIF').decode('utf-8')
            return mock_jpeg
    
    def close(self):
        if self.camera:
            self.camera.stop()
            logger.info("Camera closed")

camera = CameraInterface()

# =============================================================================
# HTTP Server for PC Communication
# =============================================================================
class RPIRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Suppress default HTTP logging, use our logger
        pass
    
    def _send_json(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode('utf-8'))
    
    def do_GET(self):
        """Handle GET requests"""
        parsed = urlparse(self.path)
        
        if parsed.path == '/status':
            # Return current state
            self._send_json(state.get_state_dict())
        
        elif parsed.path == '/obstacles':
            # Return obstacle list
            self._send_json({'obstacles': state.obstacles})
        
        else:
            self._send_json({'error': 'Unknown endpoint'}, 404)
    
    def do_POST(self):
        """Handle POST requests"""
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8')
        
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError:
            self._send_json({'error': 'Invalid JSON'}, 400)
            return
        
        parsed = urlparse(self.path)
        
        if parsed.path == '/start_task':
            # Initialize task
            state.task_start_time = time.time()
            state.task_active = True
            state.identified_count = 0
            logger.info("Task started")
            self._send_json({'status': 'started'})
        
        elif parsed.path == '/stm_command':
            # Execute STM command
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
            # Capture image for recognition
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
            # Update obstacle with recognized image
            obstacle_id = data.get('obstacle_id')
            image_id = data.get('image_id')
            
            if state.update_obstacle_image(obstacle_id, image_id):
                self._send_json({'status': 'ok'})
            else:
                self._send_json({'error': 'Obstacle not found'}, 404)
        
        elif parsed.path == '/stop_task':
            # End task
            state.task_active = False
            elapsed = time.time() - state.task_start_time if state.task_start_time else 0
            logger.info(f"Task stopped. Elapsed: {elapsed:.1f}s, Identified: {state.identified_count}")
            self._send_json({'status': 'stopped', 'elapsed': elapsed})
        
        else:
            self._send_json({'error': 'Unknown endpoint'}, 404)

def start_http_server():
    """Start HTTP server for PC communication"""
    server = HTTPServer((WLAN_HOST, WLAN_PORT), RPIRequestHandler)
    logger.info(f"HTTP server listening on {WLAN_HOST}:{WLAN_PORT}")
    server.serve_forever()

# =============================================================================
# Bluetooth Server for Android Communication
# =============================================================================
class BluetoothServer:
    def __init__(self):
        self.server_sock = None
        self.client_sock = None
        
    def start(self):
        """Start Bluetooth server"""
        if not BLUETOOTH_AVAILABLE:
            logger.warning("Bluetooth not available, skipping BT server")
            return
        
        try:
            self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.server_sock.bind(("", bluetooth.PORT_ANY))
            self.server_sock.listen(1)
            
            port = self.server_sock.getsockname()[1]
            uuid = "00001101-0000-1000-8000-00805F9B34FB"
            
            bluetooth.advertise_service(
                self.server_sock, BLUETOOTH_NAME,
                service_id=uuid,
                service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                profiles=[bluetooth.SERIAL_PORT_PROFILE]
            )
            
            logger.info(f"Bluetooth server waiting on RFCOMM channel {port}")
            
            # Accept connection
            self.client_sock, client_info = self.server_sock.accept()
            logger.info(f"Bluetooth connected to {client_info}")
            
            # Start message handler
            threading.Thread(target=self._handle_messages, daemon=True).start()
            
        except Exception as e:
            logger.error(f"Bluetooth server failed: {e}")
    
    def _handle_messages(self):
        """Handle incoming Bluetooth messages from Android"""
        buffer = ""
        while self.client_sock:
            try:
                data = self.client_sock.recv(1024).decode('utf-8')
                if not data:
                    break
                
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self._process_message(line.strip())
                    
            except Exception as e:
                logger.error(f"Bluetooth error: {e}")
                break
        
        logger.info("Bluetooth disconnected")
    
    def _process_message(self, msg):
        """Process message from Android"""
        try:
            data = json.loads(msg)
            msg_type = data.get('type')
            
            if msg_type == 'init_obstacles':
                # Receive obstacle positions from Android
                obstacles = data.get('obstacles', [])
                with state.lock:
                    state.obstacles = obstacles
                logger.info(f"Received {len(obstacles)} obstacles from Android")
                self.send_message({'type': 'ack', 'status': 'ok'})
            
            elif msg_type == 'get_status':
                # Send current status to Android
                self.send_message({
                    'type': 'status',
                    'data': state.get_state_dict()
                })
        
        except json.JSONDecodeError:
            logger.warning(f"Invalid JSON from Android: {msg}")
    
    def send_message(self, data):
        """Send message to Android"""
        if self.client_sock:
            try:
                msg = json.dumps(data) + '\n'
                self.client_sock.send(msg.encode('utf-8'))
            except Exception as e:
                logger.error(f"Failed to send BT message: {e}")
    
    def close(self):
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()
        logger.info("Bluetooth closed")

bt_server = BluetoothServer()

# =============================================================================
# Task Monitor (Timeout Watchdog)
# =============================================================================
def task_monitor():
    """Monitor task timeout"""
    while True:
        if state.task_active and state.task_start_time:
            elapsed = time.time() - state.task_start_time
            if elapsed > TASK_TIMEOUT:
                logger.warning(f"Task timeout ({TASK_TIMEOUT}s exceeded)")
                state.task_active = False
                # Send stop command to STM
                stm.send_command('s')
        time.sleep(1)

# =============================================================================
# Main
# =============================================================================
def main():
    logger.info("=== RPI MDP Server Starting ===")
    
    # Connect to STM
    if not stm.connect():
        logger.error("Failed to connect to STM, exiting")
        return
    
    # Start Bluetooth server (non-blocking)
    threading.Thread(target=bt_server.start, daemon=True).start()
    
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
        bt_server.close()
        logger.info("Shutdown complete")

if __name__ == '__main__':
    main()
