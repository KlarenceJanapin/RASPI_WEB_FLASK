import json
import os
import time
import threading
import queue
import requests
from datetime import datetime

# ============================================
# CONFIGURATION - CHANGE THIS!
# ============================================
LAPTOP_IP = "172.20.10.12"  # <-- CHANGE TO YOUR LAPTOP'S IP ADDRESS
LAPTOP_PORT = 8000
LAPTOP_URL = f"http://{LAPTOP_IP}:{LAPTOP_PORT}"

# Serial configuration
SERIAL_PORT = os.environ.get("USV_SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD = int(os.environ.get("USV_SERIAL_BAUD", "115200"))
SERIAL_TIMEOUT_S = float(os.environ.get("USV_SERIAL_TIMEOUT_S", "0.1"))

# ============================================
# Setup serial (with error handling)
# ============================================
try:
    import serial  # type: ignore
    SERIAL_AVAILABLE = True
except Exception as e:
    serial = None
    SERIAL_AVAILABLE = False
    print(f"WARNING: pyserial not installed: {e}")
    print("Install with: pip install pyserial")

# Command queue for receiving from laptop
cmd_queue = queue.Queue()

# ============================================
# Network functions
# ============================================
def send_telemetry_to_laptop(data):
    """Send telemetry data to laptop server"""
    try:
        response = requests.post(
            f"{LAPTOP_URL}/api/telemetry",
            json=data,
            timeout=1
        )
        return response.ok
    except Exception as e:
        print(f"Failed to send telemetry: {e}")
        return False

def fetch_commands_from_laptop():
    """Get pending commands from laptop server"""
    try:
        response = requests.get(
            f"{LAPTOP_URL}/api/commands/pending",
            timeout=1
        )
        if response.ok:
            commands = response.json().get('commands', [])
            for cmd in commands:
                cmd_queue.put(cmd)
                print(f"Received command from laptop: {cmd}")
            return True
    except Exception as e:
        print(f"Failed to fetch commands: {e}")
    return False

# ============================================
# Serial worker thread
# ============================================
def serial_worker():
    """Read from serial and send to laptop"""
    if not SERIAL_AVAILABLE:
        print("Serial module not available - running in demo mode")
        demo_mode()
        return
    
    # Connect to serial device
    while True:
        try:
            ser = serial.Serial(
                SERIAL_PORT,
                SERIAL_BAUD,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=SERIAL_TIMEOUT_S
            )
            print(f"✓ Connected to serial port: {SERIAL_PORT}")
            break
        except Exception as e:
            print(f"Serial connection failed: {e}, retrying in 2s...")
            time.sleep(2)
    
    rx_buf = bytearray()
    last_command_fetch = 0
    last_heartbeat = 0
    
    while True:
        try:
            # Read from serial
            b = ser.read(256)
            if b:
                rx_buf.extend(b)
                while True:
                    nl = rx_buf.find(b"\n")
                    if nl < 0:
                        break
                    line = rx_buf[:nl].decode("utf-8", errors="ignore").strip()
                    del rx_buf[:nl + 1]
                    if not line:
                        continue
                    
                    try:
                        telemetry_data = json.loads(line)
                        if isinstance(telemetry_data, dict):
                            # Add timestamp and send to laptop
                            telemetry_data['pi_timestamp'] = time.time()
                            if send_telemetry_to_laptop(telemetry_data):
                                # Only print occasionally to avoid spam
                                if telemetry_data.get('type') == 'laser_status':
                                    print(f"✓ Laser status: {'ON' if telemetry_data.get('laser_on') else 'OFF'} (angle: {telemetry_data.get('angle')}°)")
                                elif telemetry_data.get('type') == 'telem':
                                    print(f"✓ Telemetry: lat={telemetry_data.get('lat', '?')}, lng={telemetry_data.get('lng', '?')}, laser={telemetry_data.get('laserAngle', '?')}°")
                                elif telemetry_data.get('type') == 'ack':
                                    print(f"✓ ACK from ESP32: {telemetry_data}")
                    except json.JSONDecodeError as e:
                        print(f"JSON decode error: {e}, line: {line[:100]}")
            
            # Fetch commands from laptop every 100ms
            if time.time() - last_command_fetch > 0.1:
                fetch_commands_from_laptop()
                last_command_fetch = time.time()
            
            # Send heartbeat every 5 seconds
            if time.time() - last_heartbeat > 5:
                heartbeat = {"type": "heartbeat", "timestamp": time.time()}
                ser.write((json.dumps(heartbeat) + "\n").encode('utf-8'))
                last_heartbeat = time.time()
            
            # Process and send commands to serial
            try:
                cmd = cmd_queue.get_nowait()
                # Convert command to JSON line and send to serial
                cmd_line = json.dumps(cmd) + "\n"
                bytes_written = ser.write(cmd_line.encode('utf-8'))
                ser.flush()
                print(f"✓ Sent command to USV: {cmd} ({bytes_written} bytes)")
                # If it's a laser command, also send acknowledgment back to laptop
                if cmd.get('cmd') in ['laser_on', 'laser_off', 'laser_angle']:
                    status = cmd.get('cmd')
                    print(f"  → Laser command sent: {status}")
            except queue.Empty:
                pass
            except Exception as e:
                print(f"Error sending command: {e}")
                
        except Exception as e:
            print(f"Serial worker error: {e}")
            time.sleep(1)

# ============================================
# Demo mode (for testing without hardware)
# ============================================
def demo_mode():
    """Generate fake telemetry for testing"""
    print("Running in DEMO MODE - generating fake telemetry")
    import random
    
    while True:
        # Generate fake telemetry data
        fake_data = {
            "type": "telem",
            "lat": 37.7749 + random.uniform(-0.01, 0.01),
            "lng": -122.4194 + random.uniform(-0.01, 0.01),
            "head": random.uniform(0, 360),
            "dist": random.uniform(0, 100),
            "waterPH": random.uniform(6.5, 8.5),
            "waterTDS": random.uniform(100, 500),
            "waterTemp": random.uniform(15, 25),
            "rssi": random.randint(-90, -40),
            "snr": random.uniform(5, 30),
            "laserAngle": random.choice([0, 20, 45, 90]),
            "laserState": random.choice([0, 1]),
            "demo_mode": True
        }
        
        # Send to laptop
        send_telemetry_to_laptop(fake_data)
        print(f"✓ Sent demo telemetry (laser: {fake_data['laserState']})")
        
        # Fetch commands
        fetch_commands_from_laptop()
        
        # Process any queued commands (simulate)
        try:
            cmd = cmd_queue.get_nowait()
            print(f"Demo mode - would send: {cmd}")
        except queue.Empty:
            pass
        
        time.sleep(1)

# ============================================
# Main
# ============================================
def main():
    print("=" * 50)
    print("Raspberry Pi Telemetry Sender (with Laser Support)")
    print("=" * 50)
    print(f"Laptop server: {LAPTOP_URL}")
    print(f"Serial port: {SERIAL_PORT}")
    print(f"Baud rate: {SERIAL_BAUD}")
    print("=" * 50)
    
    # Test connection to laptop
    print("Testing connection to laptop...")
    try:
        response = requests.get(f"{LAPTOP_URL}/", timeout=2)
        if response.status_code == 200:
            print("✓ Connected to laptop server!")
        else:
            print("⚠ Connected but unexpected response")
    except Exception as e:
        print(f"✗ Cannot connect to laptop: {e}")
        print(f"Make sure laptop server is running and IP is correct!")
        print(f"Current laptop IP configured: {LAPTOP_IP}")
        return
    
    # Start serial worker thread
    serial_thread = threading.Thread(target=serial_worker, daemon=True)
    serial_thread.start()
    print("✓ Serial worker started")
    
    # Keep running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        print("Goodbye!")

if __name__ == "__main__":
    main()
