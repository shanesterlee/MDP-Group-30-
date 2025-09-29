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
    """Handle one chat session with Android"""
    print(f"[RPi] Connected to {client_info}")
    stop_flag = [False]

    def receive_messages(sock, stop_flag):
        while not stop_flag[0]:
            try:
                data = sock.recv(1024).decode("utf-8")
                if not data:
                    print("\n[Android disconnected]")
                    stop_flag[0] = True
                    break
                if data.strip().lower() in ["exit", "quit"]:
                    print("\n[Android ended the chat]")
                    stop_flag[0] = True
                    break

                print(f"\n[Android] {data}")

                # Handle STM command mapping
                if handle_android_command(data):
                    print(f"[RPi] Forwarded Android cmd → STM")
                else:
                    print("[RPi] No mapping for this message")

            except OSError:
                print("\n[Android connection lost]")
                stop_flag[0] = True
                break

    # Receiver thread
    recv_thread = threading.Thread(target=receive_messages, args=(client_sock, stop_flag))
    recv_thread.daemon = True
    recv_thread.start()

    try:
        while not stop_flag[0]:
            msg = input("[RPi] > ")
            if msg.strip().lower() in ["exit", "quit"]:
                try:
                    client_sock.send("[RPi disconnected]".encode("utf-8"))
                except:
                    pass
                print("[RPi ended the chat]")
                stop_flag[0] = True
                break
            client_sock.send(msg.encode("utf-8"))
    except KeyboardInterrupt:
        print("\n[RPi] Chat interrupted")
        stop_flag[0] = True

    try:
        client_sock.shutdown(2)
    except:
        pass
    client_sock.close()
    print("[RPi] Session closed\n")

# ----------------------------
# Main Bluetooth Server
# ----------------------------
def main():
    global running
    server_sock = BluetoothSocket(RFCOMM)
    server_sock.bind(("", PORT_ANY))
    server_sock.listen(1)

    port = server_sock.getsockname()[1]
    uuid = "00001101-0000-1000-8000-00805F9B34FB"

    advertise_service(
        server_sock,
        "RPI-BT-Chat",
        service_id=uuid,
        service_classes=[uuid, SERIAL_PORT_CLASS],
        profiles=[SERIAL_PORT_PROFILE],
    )

    print(f"[RPi] Bluetooth chat server running on RFCOMM channel {port}...")
    print("[RPi] Press CTRL+C anytime to quit the server.\n")

    try:
        while running:
            print("[RPi] Waiting for connection...")
            try:
                client_sock, client_info = server_sock.accept()
                chat_session(client_sock, client_info)
                # after session ends, loop back and wait again
            except OSError:
                break
    finally:
        server_sock.close()
        print("[RPi] Server shut down")

if __name__ == "__main__":
    main()

