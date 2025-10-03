#!/usr/bin/env python3
import time, threading, requests
from flask import Flask, jsonify
from bluetooth import *

# ----------------------------
# CONFIG
# ----------------------------
PC_URL = "http://192.168.30.15:5005/from_rpi"   # PC Algorithm Server endpoint
RPi_PORT = 5000
running = True

# ----------------------------
# Global State
# ----------------------------
objects = [[-1, -1, "N"] for _ in range(8)]  # 8 slots default
last_msg = None
last_reply = None

# ----------------------------
# Flask Setup
# ----------------------------
app = Flask(__name__)

@app.route("/status", methods=["GET"])
def status():
    return jsonify({"status": "RPi Server online"})

@app.route("/get_last_msg", methods=["GET"])
def get_last_msg():
    if last_msg:
        return jsonify({"last_msg": last_msg}), 200
    return jsonify({"error": "no message yet"}), 404

@app.route("/get_last_reply", methods=["GET"])
def get_last_reply():
    if last_reply:
        return jsonify({"last_reply": last_reply}), 200
    return jsonify({"error": "no reply yet"}), 404

# ----------------------------
# Helpers
# ----------------------------
def forward_to_pc(payload):
    """Send payload to PC algorithm server"""
    global last_reply
    try:
        print(f"[RPi -> PC] Sending: {payload}")
        resp = requests.post(PC_URL, json=payload, timeout=10)
        if resp.ok:
            last_reply = resp.json()
            print(f"[PC -> RPi] Response: {last_reply}")
        else:
            print(f"[ERROR] PC API error {resp.status_code}: {resp.text}")
    except Exception as e:
        print(f"[ERROR] Could not contact PC API: {e}")

def update_object(msg):
    """Parse OBJECT commands or handle Y from Android"""
    global objects

    if msg.strip().upper() == "Y":
        # Finalize and send all objects
        payload = {"objects": objects}
        forward_to_pc(payload)
        return

    # Example: OBJECT4, 12, 3, S
    try:
        parts = [p.strip() for p in msg.split(",")]
        if len(parts) != 4:
            print(f"[WARN] Invalid format: {msg}")
            return

        obj_str, x, y, d = parts
        if not obj_str.upper().startswith("OBJECT"):
            print(f"[WARN] Unknown command: {msg}")
            return

        idx = int(obj_str.replace("OBJECT", "")) - 1  # OBJECT4 -> index 3
        if 0 <= idx < 8:
            objects[idx] = [int(x), int(y), d.upper()]
            print(f"[RPi] Updated {obj_str}: {objects[idx]}")
        else:
            print(f"[WARN] Invalid object index: {obj_str}")

    except Exception as e:
        print(f"[ERROR] Failed to parse object msg '{msg}': {e}")

# ----------------------------
# Bluetooth Handling
# ----------------------------
def chat_session(client_sock, client_info):
    global last_msg
    print(f"[BT] Connected to {client_info}")
    try:
        while True:
            data = client_sock.recv(1024).decode("utf-8").strip()
            if not data:
                break
            last_msg = data
            print(f"[Android -> RPi] {data}")
            update_object(data)
    except Exception as e:
        print(f"[BT ERROR] {e}")
    finally:
        client_sock.close()
        print("[BT] Session closed")

def bluetooth_server():
    uuid = "00001101-0000-1000-8000-00805F9B34FB"
    while running:
        try:
            server_sock = BluetoothSocket(RFCOMM)
            server_sock.bind(("", PORT_ANY))
            server_sock.listen(1)

            port = server_sock.getsockname()[1]
            advertise_service(
                server_sock,
                "RPI-BT-Chat",
                service_id=uuid,
                service_classes=[uuid, SERIAL_PORT_CLASS],
                profiles=[SERIAL_PORT_PROFILE],
            )

            print(f"[BT] Listening on RFCOMM channel {port}...")
            client_sock, client_info = server_sock.accept()
            chat_session(client_sock, client_info)

        except Exception as e:
            print(f"[BT ERROR] {e}, retrying in 2s...")
            time.sleep(2)
        finally:
            try:
                server_sock.close()
            except:
                pass

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    # Start Flask server in background
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=RPi_PORT, debug=False),
        daemon=True
    )
    flask_thread.start()

    # Run Bluetooth listener in main thread
    bluetooth_server()
