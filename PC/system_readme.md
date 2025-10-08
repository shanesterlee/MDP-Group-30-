# MDP Robot Navigation System - Complete Setup Guide

## System Architecture

```
┌─────────────┐     Bluetooth      ┌──────────────────┐
│   Android   │ ◄─────────────────► │   Raspberry Pi   │
│     App     │                     │     (RPI)        │
└─────────────┘                     └──────────────────┘
                                           │ │ │
                                           │ │ └─► Camera
                      ┌────────────────────┘ │
                      │ WLAN                 │ Serial (UART)
                      ▼                      ▼
                ┌──────────┐          ┌──────────┐
                │    PC    │          │   STM    │
                │ Planner  │          │  Robot   │
                └──────────┘          └──────────┘
```

## Components Overview

### 1. **Android App** (Not included - hooks provided)
- Sends obstacle positions/orientations to RPI via Bluetooth
- Displays identified images and robot position on map
- Monitors task progress

### 2. **Raspberry Pi (RPI)**
- **File**: `rpi_server.py`
- Receives obstacle data from Android via Bluetooth
- Hosts HTTP server for PC communication
- Relays STM commands from PC to STM via serial
- Captures images using Pi Camera
- Tracks robot position

### 3. **PC Path Planner**
- **Files**: `pc_client_full.py`, `planner_integration.py`
- Fetches obstacle positions from RPI
- Plans optimal Dubins paths (Ackermann constraints)
- Generates STM command sequences
- Sends commands to RPI → STM
- Processes images for recognition
- Coordinates entire task execution

### 4. **STM32 Robot**
- **File**: `main(STM).c`
- Executes movement commands: `w`, `x`, `a`, `d`, `z`, `c`, `s`, `t`, `m`
- Reports position after each movement: `heading, x, y`
- Controls motors, encoders, gyroscope, ultrasonic sensor

### 5. **Vision Recognition**
- **File**: `vision.py`
- Identifies images on obstacle faces
- Uses Roboflow SDK or local server
- Maps detected classes to image IDs (11-40)

### 6. **Simulator** (Testing/Development)
- **File**: `mdp_2_sim.py`
- Visual simulation of robot and obstacles
- Path planning algorithms (Dubins + lattice A*)
- Used for testing path planning logic

---

## Installation

### Raspberry Pi Setup

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-opencv
sudo apt-get install -y libcamera-dev python3-picamera2
sudo apt-get install -y bluetooth bluez libbluetooth-dev

# Install Python packages
pip3 install pyserial requests

# For Bluetooth support
pip3 install pybluez

# Enable serial on GPIO (if using GPIO UART)
sudo raspi-config
# Navigate to: Interface Options → Serial Port
# Enable serial, disable login shell

# Set up camera
sudo raspi-config
# Navigate to: Interface Options → Camera → Enable
```

### PC Setup

```bash
# Install Python packages
pip3 install requests opencv-python shapely

# For vision recognition
pip3 install inference-sdk  # Roboflow SDK

# For simulator (optional, for testing)
pip3 install pygame shapely
```

### STM32 Setup

1. Flash `main(STM).c` to STM32F4 board using STM32CubeIDE
2. Connect via USB serial (appears as `/dev/ttyUSB0` on Linux)
3. Verify baud rate: 115200

---

## Hardware Connections

### RPI to STM
- **USB Serial**: `/dev/ttyUSB0` (or `/dev/ttyACM0`)
- **OR GPIO UART**: TX (GPIO14) ↔ RX, RX (GPIO15) ↔ TX, GND ↔ GND

### RPI Camera
- Connect Pi Camera Module to CSI port

### STM Robot
- Motors connected to TIM4 (CH3, CH4) and TIM9 (CH1, CH2)
- Encoders on TIM2 and TIM3
- Gyroscope (ICM-20948) on I2C2
- Ultrasonic sensor: TRIG on PC8, ECHO on PC7
- Servo on TIM12 CH1

---

## Configuration

### RPI Server (`rpi_server.py`)

Edit configuration at top of file:

```python
WLAN_HOST = '0.0.0.0'  # Listen on all interfaces
WLAN_PORT = 5000       # HTTP server port
STM_PORT = '/dev/ttyUSB0'  # or '/dev/serial0' for GPIO UART
STM_BAUD = 115200
TASK_TIMEOUT = 300  # 5 minutes
```

### PC Client (`pc_client_full.py`)

Command-line arguments:

```bash
--rpi-host 192.168.30.1.100    # RPI IP address
--rpi-port 5000             # RPI HTTP port
--vision-server URL         # Optional: local vision server
--max-retries 1             # Image recognition retries
--timeout 300               # Task timeout (seconds)
```

### Vision (`vision.py`)

Edit at top of file for Roboflow credentials:

```python
API_URL  = "https://serverless.roboflow.com"
API_KEY  = "your_roboflow_api_key"
MODEL_ID = "your_model_id"
```

---

## Running the System

### Step 1: Start RPI Server

On Raspberry Pi:

```bash
python3 rpi_server.py
```

Expected output:
```
[12:00:00] INFO: Connected to STM on /dev/ttyUSB0 @ 115200
[12:00:00] INFO: Camera initialized
[12:00:00] INFO: HTTP server listening on 0.0.0.0:5000
[12:00:00] INFO: Bluetooth server waiting on RFCOMM channel 1
```

### Step 2: Connect Android App

1. Open Android app
2. Connect to "RPI_MDP_SERVER" via Bluetooth
3. Enter obstacle positions and orientations:
   - Obstacle A: (0.40, 1.55) facing S
   - Obstacle B: (1.55, 1.40) facing W
   - Obstacle C: (0.30, 0.50) facing E
4. Send obstacles to RPI

### Step 3: Start PC Client

On PC (connected to same network as RPI):

```bash
# Find RPI IP address first
# On RPI: hostname -I

# Run PC client
python3 pc_client_full.py --rpi-host 192.168.1.100
```

Expected output:
```
======================================================================
PC CLIENT - MDP ROBOT NAVIGATION SYSTEM
======================================================================
RPI:         192.168.1.100:5000
Vision:      Roboflow SDK
Max Retries: 1
Timeout:     300s
Planning:    Enabled
======================================================================

⏳ Waiting for obstacles from Android...
✓ Received 3 obstacles from Android:

  1. A: (0.40m, 1.55m) facing S
  2. B: (1.55m, 1.40m) facing W
  3. C: (0.30m, 0.50m) facing E

──────────────────────────────────────────────────────────────────────
Press ENTER to start task execution...
```

### Step 4: Execute Task

Press ENTER to start. The system will:

1. **Plan optimal path** using Dubins curves
2. **Navigate to each obstacle** in optimal order
3. **Capture image** at each obstacle
4. **Recognize image** using vision API
5. **Update map** with identified images
6. **Continue until all done** or timeout (5 min)

---

## STM Command Reference

| Command | Description | Example |
|---------|-------------|---------|
| `w<dist>` | Move forward (cm) | `w50` = 50cm forward |
| `x<dist>` | Move backward (cm) | `x30` = 30cm back |
| `a<angle>` | Turn left (degrees) | `a45` = turn left 45° |
| `d<angle>` | Turn right (degrees) | `d90` = turn right 90° |
| `z<angle>` | Reverse turn left | `z45` |
| `c<angle>` | Reverse turn right | `c45` |
| `s` | Stop motors | `s` |
| `t<pos>` | Set servo position | `t71` = center |
| `m<dist>` | Move forward (no tracking) | `m5` = for image adjust |

**Response Format**: `heading, x, y` (e.g., `90, 125, 100`)

---

## API Reference

### RPI HTTP Endpoints

#### GET `/status`
Returns current system state

**Response**:
```json
{
  "obstacles": [...],
  "robot_pos": {"x": 0.125, "y": 0.1, "heading": 90},
  "task_active": true,
  "identified_count": 2,
  "elapsed_time": 45.2
}
```

#### GET `/obstacles`
Returns obstacle list

**Response**:
```json
{
  "obstacles": [
    {"id": "A", "x": 0.40, "y": 1.55, "face": "S", "image_id": null},
    ...
  ]
}
```

#### POST `/start_task`
Initialize task

**Response**: `{"status": "started"}`

#### POST `/stm_command`
Send command to STM

**Request**:
```json
{"command": "w50"}
```

**Response**:
```json
{
  "status": "ok",
  "response": "90, 175, 100"
}
```

#### POST `/capture`
Capture image

**Request**:
```json
{"obstacle_id": "A"}
```

**Response**:
```json
{
  "status": "ok",
  "obstacle_id": "A",
  "image": "base64_encoded_jpeg_data..."
}
```

#### POST `/update_image`
Update recognized image

**Request**:
```json
{
  "obstacle_id": "A",
  "image_id": 15
}
```

**Response**: `{"status": "ok"}`

#### POST `/stop_task`
End task

**Response**:
```json
{
  "status": "stopped",
  "elapsed": 123.4
}
```

---

## Troubleshooting

### RPI Cannot Connect to STM
- Check USB cable connection
- Verify port: `ls /dev/tty*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches (115200)

### PC Cannot Reach RPI
- Verify network connectivity: `ping 192.168.1.100`
- Check firewall: `sudo ufw allow 5000`
- Verify RPI IP: On RPI run `hostname -I`

### Bluetooth Connection Fails
- Check Bluetooth is enabled: `sudo systemctl status bluetooth`
- Make device discoverable: `sudo bluetoothctl` → `discoverable on`
- Pair device first from Android settings

### Camera Not Working
- Enable camera: `sudo raspi-config`
- Test camera: `libcamera-hello --timeout 5000`
- Check connection to CSI port

### Vision Recognition Fails
- Check API key in `vision.py`
- Test API: `python vision.py test_image.jpg out.jpg`
- Verify image quality (lighting, focus, distance)

### Path Planning Fails
- Check obstacle positions are valid (within 2m × 2m arena)
- Reduce standoff distance if too conservative
- Increase Rmin if turns too tight
- Check inflation radius isn't excessive

### Robot Position Drift
- Calibrate encoders: adjust `ENCODER_SCALE_CORR` in STM code
- Calibrate gyroscope: verify `GYRO_SCALE_CORR`
- Check wheel slippage
- Verify floor surface is suitable

---

## Testing

### Test RPI Server Locally
```bash
python3 rpi_server.py
# In another terminal:
curl http://localhost:5000/status
```

### Test Vision Recognition
```bash
python3 vision.py test_image.jpg output.jpg --conf 0.3
```

### Test Simulator
```bash
python3 mdp_2_sim.py
# Press 'P' to plan, 'A' to animate
```

### Test STM Serial
```python
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200)
ser.write(b'w10\n')  # Move 10cm forward
print(ser.readline())  # Read response
```

---

## Performance Tips

1. **Optimize Visit Order**: Use TSP solver for better order
2. **Parallel Processing**: Capture next image while moving
3. **Reduce Retries**: Improve lighting/camera position
4. **Faster Recognition**: Use local vision server
5. **Smoother Paths**: Increase path sampling density
6. **Better Localization**: Add AprilTags or visual odometry

---

## Known Limitations

- **5-minute timeout**: Hard limit, task stops after 300s
- **No dynamic re-planning**: Path fixed at start
- **Serial communication**: Blocking, no parallel execution
- **Simple collision checking**: Basic geometric checks only
- **Position drift**: Accumulates over long runs
- **Single retry**: Only one re-attempt per obstacle

---

## Future Enhancements

- [ ] Android app integration (Bluetooth hooks ready)
- [ ] Dynamic obstacle avoidance
- [ ] SLAM for better localization
- [ ] Multi-robot coordination
- [ ] Web dashboard for monitoring
- [ ] Logging and replay capabilities
- [ ] Simulation integration for testing

---

## License & Credits

Developed for MDP (Multidisciplinary Project) course.

**Robot**: Ackermann steering vehicle (25cm × 20cm)
- Max steering angle: 45°
- Wheelbase: 15cm
- Min turning radius: ~15cm

**Components**:
- STM32F4 microcontroller
- Raspberry Pi 4 Model B
- Pi Camera Module v2
- ICM-20948 IMU
- HC-SR04 ultrasonic sensor

---

## Contact & Support

For issues, questions, or contributions, please refer to the project documentation or contact the development team.

**Remember**: Always test in simulation before running on real robot!
