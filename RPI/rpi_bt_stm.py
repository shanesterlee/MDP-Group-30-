#Code to talk between RPI , Android and STM . Used for A1 
#Hosted on RPI
#!/usr/bin/env python3
from bluetooth import *
import threading
import sys
import signal
import serial
import time

# ----------------------------
# Global flags
# ----------------------------
running = True

# ----------------------------
# Setup STM serial connection
# ----------------------------
try:
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=1)
    time.sleep(2)  # STM resets on port open
    print("[STM] Connected on /dev/ttyACM0")
except Exception as e:
    print(f"[ERROR] Could not open STM serial: {e}")
    sys.exit(1)

# ----------------------------
# Signal handler for CTRL+C
# ----------------------------
def signal_handler(sig, frame):
    global running
    print("\n[RPi] Shutting down server...")
    running = False
    try:
        ser.close()
    except:
        pass
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ----------------------------
# Command Mapping
# ----------------------------
def handle_android_command(msg: str):
    """Map Android messages to STM commands."""
    text = msg.strip().upper()
    if text == "ROBOT FORWARD":
        ser.write(b"w20\n")
        print("[STM] Sent 'w20' to move forward")
        return True
    # add more mappings here if needed
    return False

# ----------------------------
# Chat Session
# ----------------------------
def chat_session(client_sock, client_info):
    print(f"[RPi] Connected to {client_info}")

    def receive_messages(sock):
        while True:
            try:
                data = sock.recv(1024).decode("utf-8")
                if not data:
                    print("[Android disconnected]")
                    break

                print(f"[Android] {data}")

                if handle_android_command(data):
                    print("[RPi] Forwarded Android cmd → STM")
                else:
                    print("[RPi] No mapping for this message")

            except (OSError, IOError):
                print("[RPi] Connection lost")
                break

        try:
            sock.close()
        except:
            pass
        print("[RPi] Session closed")

    # Run receiver thread
    recv_thread = threading.Thread(target=receive_messages, args=(client_sock,))
    recv_thread.daemon = True
    recv_thread.start()

    # Keep main loop alive until receiver ends
    while recv_thread.is_alive():
        time.sleep(0.5)

# ----------------------------
# Main Bluetooth Server
# ----------------------------
def main():
    global running
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

            print(f"[RPi] Listening on RFCOMM channel {port}...")

            client_sock, client_info = server_sock.accept()
            chat_session(client_sock, client_info)

        except Exception as e:
            print(f"[ERROR] {e}, retrying in 2s...")
            time.sleep(2)
        finally:
            try:
                server_sock.close()
            except:
                pass

if __name__ == "__main__":
    main()
