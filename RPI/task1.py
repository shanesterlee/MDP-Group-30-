#Used for task A5. Takes picture and gets image recognition from PC via the model.
#Hosted on RPI
import cv2
import os
import serial
import time
import threading
import subprocess
from flask import Flask, Response, send_file, request, jsonify
import serial.serialutil
import json

# ----------------------------
# Kill conflicting processes
# ----------------------------
def cleanup():
    print("[SYS] Cleaning up old processes...")
    try:
        subprocess.run(["sudo", "fuser", "-k", "/dev/ttyACM0"], check=False)
        subprocess.run(["sudo", "fuser", "-k", "5000/tcp"], check=False)
        subprocess.run(["sudo", "fuser", "-k", "/dev/video0"], check=False)
    except Exception as e:
        print(f"[WARN] Cleanup failed: {e}")

cleanup()

# ----------------------------
# Flask Setup
# ----------------------------
app = Flask(__name__)
last_photo = "photo.jpg"

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("[ERROR] Could not open camera. Exiting.")
    exit(1)

# Serial port for STM
try:
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1)
    time.sleep(2)  # STM resets on port open
except Exception as e:
    print(f"[ERROR] Could not open serial port: {e}")
    exit(1)

# ----------------------------
# Camera Functions
# ----------------------------
def generate_frames():
    """Stream live camera frames as MJPEG."""
    while True:
        success, frame = cap.read()
        if not success:
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# ----------------------------
# Algo Instructions Endpoint
# ----------------------------
@app.route('/algo_instructions', methods=['POST'])
def algo_instructions():
    """
    Receives JSON with decision from PC/algorithm.
    If decision is 'snapshot', captures photo and returns it.
    Otherwise, forwards decision to STM and returns STM's response.
    
    Expected JSON format: {"decision": "snapshot"} or {"decision": "forward/left/right/etc"}
    """
    try:
        # Parse incoming JSON
        data = request.get_json()
        if not data or 'decision' not in data:
            return jsonify({"error": "Missing 'decision' field in JSON"}), 400
        
        decision = data['decision'].strip().lower()
        print(f"[ALGO] Received decision: {decision}")
        
        # Case 1: Snapshot request
        if decision == "snapshot":
            success, frame = cap.read()
            if success:
                cv2.imwrite(last_photo, frame)
                print(f"[ALGO] Snapshot captured as {last_photo}")
                # Return the photo file
                return send_file(last_photo, mimetype='image/jpeg')
            else:
                return jsonify({"error": "Failed to capture photo"}), 500
        
        # Case 2: Forward to STM
        else:
            try:
                # Send decision to STM
                ser.write((decision + "\n").encode())
                print(f"[ALGO] Forwarded to STM: {decision}")
                
                # Wait for STM response (with timeout)
                start_time = time.time()
                timeout = 5  # 5 second timeout
                stm_response = ""
                
                while time.time() - start_time < timeout:
                    if ser.in_waiting > 0:
                        stm_response = ser.readline().decode(errors="ignore").strip()
                        if stm_response:
                            print(f"[ALGO] STM responded: {stm_response}")
                            break
                    time.sleep(0.1)
                
                if not stm_response:
                    return jsonify({
                        "message": "STM did not respond in time"
                    }), 408
                
                # Return STM's response as JSON
                return jsonify({
                    "stm_response": stm_response
                }), 200
                
            except Exception as e:
                return jsonify({"error": f"Serial communication error: {e}"}), 500
    
    except Exception as e:
        return jsonify({"error": f"Processing error: {e}"}), 500

# ----------------------------
# STM Communication Loop
# ----------------------------
def stm_loop():
    """Continuously listen to STM and forward whatever it says."""
    print("[STM] Listener started. Forwarding all STM messages...")

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                print(f"[STM] {line}")
        except serial.serialutil.SerialException as e:
            print(f"[ERROR] Serial read failed: {e}")
            time.sleep(1)
            continue

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    # Start Flask in background thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False),
        daemon=True
    )
    flask_thread.start()

    # Run STM listener in main thread
    stm_loop()