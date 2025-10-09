#!/usr/bin/env python3
"""
pc_client_full.py - Complete PC client with integrated path planning

Usage:
    python pc_client_full.py --rpi-host 192.168.1.100
"""

import requests
import json
import time
import argparse
import base64
import tempfile
import os
import sys
import math

# Import modules
try:
    from vision import call_roboflow_sdk, call_local_server, class_to_id
    VISION_AVAILABLE = True
except ImportError:
    print("[WARN] vision.py not available, using mock recognition")
    VISION_AVAILABLE = False

try:
    from planner_integration import plan_full_task, DEFAULT_STANDOFF, DEFAULT_R_MIN
    PLANNER_AVAILABLE = True
except ImportError:
    print("[WARN] planner_integration.py not available, using simple navigation")
    PLANNER_AVAILABLE = False

# =============================================================================
# Configuration
# =============================================================================
DEFAULT_RPI_HOST = '192.168.1.100'
DEFAULT_RPI_PORT = 5000
VISION_SERVER_URL = None
MAX_RETRIES = 1
RETRY_ADJUST_CM = 5
TASK_TIMEOUT = 300  # 5 minutes

# =============================================================================
# RPI Client (from previous artifact)
# =============================================================================
class RPIClient:
    def __init__(self, host, port=DEFAULT_RPI_PORT):
        self.base_url = f"http://{host}:{port}"
        self.timeout = 30
    
    def get_obstacles(self):
        try:
            r = requests.get(f"{self.base_url}/obstacles", timeout=5)
            r.raise_for_status()
            return r.json()['obstacles']
        except Exception as e:
            print(f"[ERROR] Failed to get obstacles: {e}")
            return None
    
    def get_status(self):
        try:
            r = requests.get(f"{self.base_url}/status", timeout=5)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            print(f"[ERROR] Failed to get status: {e}")
            return None
    
    def start_task(self):
        try:
            r = requests.post(f"{self.base_url}/start_task", timeout=5)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            print(f"[ERROR] Failed to start task: {e}")
            return None
    
    def send_stm_command(self, command):
        try:
            data = {'command': command}
            r = requests.post(f"{self.base_url}/stm_command", json=data, timeout=self.timeout)
            r.raise_for_status()
            result = r.json()
            if result.get('status') == 'ok':
                return result.get('response')
            else:
                print(f"[WARN] STM command failed: {result}")
                return None
        except Exception as e:
            print(f"[ERROR] STM command error: {e}")
            return None
    
    def capture_image(self, obstacle_id):
        try:
            data = {'obstacle_id': obstacle_id}
            r = requests.post(f"{self.base_url}/capture", json=data, timeout=10)
            r.raise_for_status()
            result = r.json()
            if result.get('status') == 'ok':
                return result.get('image')
            return None
        except Exception as e:
            print(f"[ERROR] Image capture failed: {e}")
            return None
    
    def update_image_id(self, obstacle_id, image_id):
        try:
            data = {'obstacle_id': obstacle_id, 'image_id': image_id}
            r = requests.post(f"{self.base_url}/update_image", json=data, timeout=5)
            r.raise_for_status()
            return r.json().get('status') == 'ok'
        except Exception as e:
            print(f"[ERROR] Failed to update image: {e}")
            return False
    
    def stop_task(self):
        try:
            r = requests.post(f"{self.base_url}/stop_task", timeout=5)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            print(f"[ERROR] Failed to stop task: {e}")
            return None

# =============================================================================
# Vision Recognition
# =============================================================================
def recognize_image(img_b64, obstacle_id):
    if not VISION_AVAILABLE:
        print("[WARN] Vision not available, returning mock ID")
        return 11  # Mock
    
    try:
        img_data = base64.b64decode(img_b64)
        with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as f:
            f.write(img_data)
            temp_path = f.name
        
        if VISION_SERVER_URL:
            result = call_local_server(temp_path, VISION_SERVER_URL)
        else:
            result = call_roboflow_sdk(temp_path)
        
        preds = result.get('predictions', [])
        if not preds:
            preds = result.get('result', {}).get('predictions', [])
        
        best_pred = None
        best_conf = 0.0
        for p in preds:
            conf = float(p.get('confidence', 0))
            if conf > best_conf:
                best_conf = conf
                best_pred = p
        
        if best_pred and best_conf >= 0.3:
            cls = best_pred.get('class', 'unknown')
            img_id = class_to_id(cls)
            print(f"[VISION] Obstacle {obstacle_id}: {cls} (conf={best_conf:.2f}) -> ID {img_id}")
            
            os.unlink(temp_path)
            
            if img_id != "NA" and isinstance(img_id, int):
                return img_id
        
        print(f"[VISION] No valid recognition for obstacle {obstacle_id}")
        os.unlink(temp_path)
        return None
    
    except Exception as e:
        print(f"[ERROR] Vision recognition failed: {e}")
        return None

# =============================================================================
# Main Task Execution with Planning
# =============================================================================
def execute_task_with_planning(rpi_client, obstacles):
    print("\n" + "="*70)
    print("TASK EXECUTION - WITH INTEGRATED PATH PLANNING")
    print("="*70)
    
    rpi_client.start_task()
    start_time = time.time()
    
    # Get current robot position
    status = rpi_client.get_status()
    if not status:
        print("[ERROR] Cannot get robot status")
        return 0, len(obstacles)
    
    robot_pos = status['robot_pos']
    start_pose = (
        robot_pos['x'],
        robot_pos['y'],
        math.radians(robot_pos['heading'])
    )
    
    print(f"\nRobot start position:")
    print(f"  ({start_pose[0]:.3f}m, {start_pose[1]:.3f}m, {math.degrees(start_pose[2]):.1f}°)")
    
    # Plan full task
    if PLANNER_AVAILABLE:
        visit_order, command_sequences = plan_full_task(start_pose, obstacles)
        if not visit_order:
            print("[ERROR] Path planning failed")
            return 0, len(obstacles)
    else:
        # Fallback: simple nearest-neighbor order
        print("[WARN] Using simple nearest-neighbor ordering")
        visit_order = [obs['id'] for obs in obstacles]
        command_sequences = None
    
    identified_count = 0
    total_obstacles = len(obstacles)
    
    # Execute each leg
    for leg_idx, obs_id in enumerate(visit_order):
        elapsed = time.time() - start_time
        if elapsed > TASK_TIMEOUT:
            print(f"\n[TIMEOUT] Task exceeded {TASK_TIMEOUT}s")
            break
        
        # Find obstacle
        obstacle = next((o for o in obstacles if o['id'] == obs_id), None)
        if not obstacle:
            print(f"[ERROR] Obstacle {obs_id} not found")
            continue
        
        print(f"\n{'─'*70}")
        print(f"LEG {leg_idx+1}/{total_obstacles}: → Obstacle {obs_id}")
        print(f"{'─'*70}")
        print(f"Target: ({obstacle['x']:.2f}, {obstacle['y']:.2f}), Face: {obstacle['face']}")
        
        # Execute commands for this leg
        if command_sequences and leg_idx < len(command_sequences):
            commands = command_sequences[leg_idx]
            print(f"Executing {len(commands)} commands: {', '.join(commands)}")
            
            for cmd in commands:
                print(f"  >> {cmd}")
                response = rpi_client.send_stm_command(cmd)
                if not response:
                    print(f"  [ERROR] Command failed: {cmd}")
                    break
                time.sleep(0.2)  # Small delay between commands
        else:
            # Fallback: navigate directly
            print("[WARN] No planned commands, using direct navigation")
            success = navigate_to_obstacle_direct(rpi_client, obstacle)
            if not success:
                print(f"[ERROR] Failed to reach obstacle {obs_id}")
                continue
        
        # Image capture and recognition
        print(f"\n📷 Capturing image of {obs_id}...")
        img_b64 = rpi_client.capture_image(obs_id)
        
        if not img_b64:
            print("[ERROR] Image capture failed")
            continue
        
        image_id = recognize_image(img_b64, obs_id)
        
        # Retry if failed
        if image_id is None and MAX_RETRIES > 0:
            print(f"🔄 Recognition failed, adjusting and retrying...")
            rpi_client.send_stm_command(f"m{RETRY_ADJUST_CM}")  # Forward move without changing position tracking
            time.sleep(0.5)
            
            img_b64 = rpi_client.capture_image(obs_id)
            if img_b64:
                image_id = recognize_image(img_b64, obs_id)
            
            rpi_client.send_stm_command(f"x{RETRY_ADJUST_CM}")  # Reverse back
            time.sleep(0.5)
        
        # Update result
        if image_id is not None:
            if rpi_client.update_image_id(obs_id, image_id):
                identified_count += 1
                print(f"✓ Obstacle {obs_id} identified as ID {image_id}")
            else:
                print(f"[ERROR] Failed to update obstacle {obs_id}")
        else:
            print(f"✗ Could not identify obstacle {obs_id}")
        
        print(f"\nProgress: {identified_count}/{total_obstacles} identified ({elapsed:.1f}s elapsed)")
    
    # Task complete
    elapsed = time.time() - start_time
    result = rpi_client.stop_task()
    
    print("\n" + "="*70)
    print("TASK EXECUTION COMPLETE")
    print("="*70)
    print(f"Total time: {elapsed:.1f}s / {TASK_TIMEOUT}s")
    print(f"Identified: {identified_count}/{total_obstacles}")
    print(f"Success rate: {100*identified_count/total_obstacles:.1f}%")
    
    if identified_count == total_obstacles:
        print("✓ All obstacles successfully identified!")
    else:
        print(f"⚠ {total_obstacles - identified_count} obstacles remain unidentified")
    
    print("="*70)
    
    return identified_count, total_obstacles

def navigate_to_obstacle_direct(rpi_client, obstacle):
    """Fallback: direct navigation without path planning"""
    standoff = DEFAULT_STANDOFF
    
    # Calculate target pose
    if obstacle['face'] == 'N':
        target_x = obstacle['x'] + 0.05
        target_y = obstacle['y'] + 0.10 + standoff
        target_heading = 270
    elif obstacle['face'] == 'S':
        target_x = obstacle['x'] + 0.05
        target_y = obstacle['y'] - standoff
        target_heading = 90
    elif obstacle['face'] == 'E':
        target_x = obstacle['x'] + 0.10 + standoff
        target_y = obstacle['y'] + 0.05
        target_heading = 180
    else:  # West
        target_x = obstacle['x'] - standoff
        target_y = obstacle['y'] + 0.05
        target_heading = 0
    
    # Get current position
    status = rpi_client.get_status()
    if not status:
        return False
    
    robot_pos = status['robot_pos']
    current_x = robot_pos['x']
    current_y = robot_pos['y']
    current_heading = robot_pos['heading']
    
    # Calculate movement
    dx = target_x - current_x
    dy = target_y - current_y
    dist = math.hypot(dx, dy)
    move_heading = math.degrees(math.atan2(dy, dx))
    
    # Turn to movement heading
    angle_diff = (move_heading - current_heading + 180) % 360 - 180
    if abs(angle_diff) > 5:
        turn_cmd = f"d{int(abs(angle_diff))}" if angle_diff > 0 else f"a{int(abs(angle_diff))}"
        print(f"  Turn: {turn_cmd}")
        if not rpi_client.send_stm_command(turn_cmd):
            return False
        time.sleep(0.3)
    
    # Move forward
    dist_cm = int(dist * 100)
    if dist_cm > 2:
        move_cmd = f"w{dist_cm}"
        print(f"  Move: {move_cmd}")
        if not rpi_client.send_stm_command(move_cmd):
            return False
        time.sleep(0.3)
    
    # Final turn to face obstacle
    status = rpi_client.get_status()
    if status:
        current_heading = status['robot_pos']['heading']
        angle_diff = (target_heading - current_heading + 180) % 360 - 180
        if abs(angle_diff) > 5:
            turn_cmd = f"d{int(abs(angle_diff))}" if angle_diff > 0 else f"a{int(abs(angle_diff))}"
            print(f"  Face: {turn_cmd}")
            if not rpi_client.send_stm_command(turn_cmd):
                return False
            time.sleep(0.3)
    
    return True

# =============================================================================
# Main Entry Point
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='PC Client for MDP Robot Navigation System',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python pc_client_full.py --rpi-host 192.168.1.100
  python pc_client_full.py --rpi-host 192.168.1.100 --vision-server http://localhost:8000/infer
  python pc_client_full.py --rpi-host 192.168.1.100 --max-retries 2
        """
    )
    parser.add_argument('--rpi-host', default=DEFAULT_RPI_HOST,
                       help=f'RPI hostname or IP (default: {DEFAULT_RPI_HOST})')
    parser.add_argument('--rpi-port', type=int, default=DEFAULT_RPI_PORT,
                       help=f'RPI HTTP port (default: {DEFAULT_RPI_PORT})')
    parser.add_argument('--vision-server', default=None,
                       help='Local vision server URL (default: use Roboflow SDK)')
    parser.add_argument('--max-retries', type=int, default=1,
                       help='Max retries for image recognition (default: 1)')
    parser.add_argument('--timeout', type=int, default=TASK_TIMEOUT,
                       help=f'Task timeout in seconds (default: {TASK_TIMEOUT})')
    args = parser.parse_args()
    
    global VISION_SERVER_URL, MAX_RETRIES, TASK_TIMEOUT
    VISION_SERVER_URL = args.vision_server
    MAX_RETRIES = args.max_retries
    TASK_TIMEOUT = args.timeout
    
    print("="*70)
    print("PC CLIENT - MDP ROBOT NAVIGATION SYSTEM")
    print("="*70)
    print(f"RPI:         {args.rpi_host}:{args.rpi_port}")
    print(f"Vision:      {'Local Server' if VISION_SERVER_URL else 'Roboflow SDK'}")
    print(f"Max Retries: {MAX_RETRIES}")
    print(f"Timeout:     {TASK_TIMEOUT}s")
    print(f"Planning:    {'Enabled' if PLANNER_AVAILABLE else 'Simple navigation'}")
    print("="*70)
    
    # Initialize RPI client
    rpi = RPIClient(args.rpi_host, args.rpi_port)
    
    # Wait for obstacles from Android
    print("\n⏳ Waiting for obstacles from Android...")
    obstacles = None
    for attempt in range(60):  # Wait up to 60 seconds
        obstacles = rpi.get_obstacles()
        if obstacles and len(obstacles) > 0:
            break
        time.sleep(1)
        if attempt % 10 == 9:
            print(f"   Still waiting... ({attempt+1}s)")
    
    if not obstacles or len(obstacles) == 0:
        print("\n❌ No obstacles received from Android. Exiting.")
        print("\nTroubleshooting:")
        print("  1. Ensure Android app is connected via Bluetooth")
        print("  2. Verify RPI Bluetooth server is running")
        print("  3. Check that obstacles were sent from Android app")
        return 1
    
    print(f"\n✓ Received {len(obstacles)} obstacles from Android:\n")
    for i, obs in enumerate(obstacles, 1):
        print(f"  {i}. {obs['id']}: ({obs['x']:.2f}m, {obs['y']:.2f}m) facing {obs['face']}")
    
    # Confirm before starting
    print("\n" + "─"*70)
    input("Press ENTER to start task execution... ")
    
    # Execute task
    try:
        identified, total = execute_task_with_planning(rpi, obstacles)
        
        if identified == total:
            print("\n🎉 SUCCESS! All obstacles identified!")
            return 0
        elif identified > 0:
            print(f"\n⚠️  PARTIAL SUCCESS: {identified}/{total} obstacles identified")
            return 1
        else:
            print(f"\n❌ FAILURE: No obstacles identified")
            return 2
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Task interrupted by user")
        rpi.stop_task()
        return 3
    
    except Exception as e:
        print(f"\n❌ Task failed with error: {e}")
        import traceback
        traceback.print_exc()
        rpi.stop_task()
        return 4

if __name__ == '__main__':
    sys.exit(main())
