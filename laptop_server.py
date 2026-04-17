import json
import os
import time
import threading
import queue
from datetime import datetime
from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# ============================================
# CONFIGURATION - CHANGE THIS!
# ============================================
LAPTOP_IP = "172.20.10.12"  # <-- CHANGE TO YOUR LAPTOP'S IP ADDRESS
LAPTOP_PORT = 8000

# Store telemetry data
telemetry_data = []
command_queue = []

# HTML Template
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>USV Control Dashboard</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        
        h1 {
            color: white;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .status-bar {
            background: white;
            border-radius: 10px;
            padding: 15px;
            margin-bottom: 20px;
            text-align: center;
            font-weight: bold;
        }
        
        .status-online {
            color: #27ae60;
        }
        
        .status-offline {
            color: #e74c3c;
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .panel {
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        
        .panel h2 {
            color: #333;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 5px;
        }
        
        .telemetry-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
        }
        
        .telemetry-item {
            background: #f5f5f5;
            padding: 10px;
            border-radius: 5px;
            text-align: center;
        }
        
        .telemetry-label {
            font-size: 0.85em;
            color: #666;
            margin-bottom: 5px;
        }
        
        .telemetry-value {
            font-size: 1.3em;
            font-weight: bold;
            color: #667eea;
        }
        
        .command-group {
            margin-bottom: 20px;
            padding: 15px;
            background: #f9f9f9;
            border-radius: 5px;
        }
        
        .command-group h3 {
            margin-bottom: 10px;
            color: #555;
        }
        
        button {
            background: #667eea;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 1em;
            margin: 5px;
            transition: all 0.3s;
        }
        
        button:hover {
            background: #764ba2;
            transform: translateY(-2px);
        }
        
        button:active {
            transform: translateY(0);
        }
        
        .btn-danger {
            background: #e74c3c;
        }
        
        .btn-danger:hover {
            background: #c0392b;
        }
        
        .btn-success {
            background: #27ae60;
        }
        
        .btn-success:hover {
            background: #229954;
        }
        
        .btn-warning {
            background: #f39c12;
        }
        
        .btn-warning:hover {
            background: #e67e22;
        }
        
        .angle-control {
            display: inline-block;
            margin-left: 10px;
        }
        
        input[type="number"] {
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 5px;
            width: 80px;
            margin: 0 5px;
        }
        
        .log-panel {
            background: white;
            border-radius: 10px;
            padding: 20px;
            max-height: 300px;
            overflow-y: auto;
        }
        
        .log-entry {
            font-family: monospace;
            font-size: 0.85em;
            padding: 5px;
            border-bottom: 1px solid #eee;
            color: #555;
        }
        
        .log-time {
            color: #667eea;
            font-weight: bold;
        }
        
        .laser-status {
            display: inline-block;
            padding: 5px 10px;
            border-radius: 20px;
            font-size: 0.9em;
            margin-left: 10px;
        }
        
        .laser-on {
            background: #ff4444;
            color: white;
        }
        
        .laser-off {
            background: #666;
            color: white;
        }
        
        @media (max-width: 768px) {
            .grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚤 USV Control Dashboard</h1>
        
        <div class="status-bar">
            <span id="connectionStatus">🔌 Connecting...</span>
            <span id="laserStatusDisplay" class="laser-status laser-off">LASER OFF</span>
        </div>
        
        <div class="grid">
            <div class="panel">
                <h2>📡 Telemetry Data</h2>
                <div class="telemetry-grid">
                    <div class="telemetry-item">
                        <div class="telemetry-label">Latitude</div>
                        <div class="telemetry-value" id="lat">---</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Longitude</div>
                        <div class="telemetry-value" id="lng">---</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Heading</div>
                        <div class="telemetry-value" id="head">---°</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Distance</div>
                        <div class="telemetry-value" id="dist">---m</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Water pH</div>
                        <div class="telemetry-value" id="waterPH">---</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Water TDS</div>
                        <div class="telemetry-value" id="waterTDS">---ppm</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Water Temp</div>
                        <div class="telemetry-value" id="waterTemp">---°C</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">RSSI</div>
                        <div class="telemetry-value" id="rssi">---dBm</div>
                    </div>
                    <div class="telemetry-item">
                        <div class="telemetry-label">Laser Angle</div>
                        <div class="telemetry-value" id="laserAngle">---°</div>
                    </div>
                </div>
            </div>
            
            <div class="panel">
                <h2>🎮 Control Panel</h2>
                
                <div class="command-group">
                    <h3>🔦 Laser Control</h3>
                    <button class="btn-success" onclick="sendLaserCommand('laser_on')">🔴 Laser ON</button>
                    <button class="btn-danger" onclick="sendLaserCommand('laser_off')">⚫ Laser OFF</button>
                    <div class="angle-control">
                        <label>Angle: </label>
                        <input type="number" id="laserAngleInput" value="20" min="0" max="180">
                        <button onclick="sendLaserAngle()">Set Angle</button>
                        <button onclick="sendLaserOnWithAngle()">ON at Angle</button>
                    </div>
                </div>
                
                <div class="command-group">
                    <h3>🎯 Navigation</h3>
                    <button onclick="sendNavigationCommand('forward')">⬆️ Forward</button>
                    <button onclick="sendNavigationCommand('backward')">⬇️ Backward</button>
                    <button onclick="sendNavigationCommand('left')">⬅️ Left</button>
                    <button onclick="sendNavigationCommand('right')">➡️ Right</button>
                    <button onclick="sendNavigationCommand('stop')">🛑 Stop</button>
                </div>
                
                <div class="command-group">
                    <h3>📊 System</h3>
                    <button onclick="sendSystemCommand('restart')">🔄 Restart System</button>
                    <button onclick="sendSystemCommand('status')">📈 Get Status</button>
                    <button onclick="clearLogs()">🗑️ Clear Logs</button>
                </div>
            </div>
        </div>
        
        <div class="log-panel">
            <h2>📝 Event Log</h2>
            <div id="logEntries">
                <div class="log-entry">System ready - waiting for data...</div>
            </div>
        </div>
    </div>
    
    <script>
        let lastUpdate = 0;
        
        function addLog(message, type = 'info') {
            const logDiv = document.getElementById('logEntries');
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = document.createElement('div');
            logEntry.className = 'log-entry';
            logEntry.innerHTML = `<span class="log-time">[${timestamp}]</span> ${message}`;
            logDiv.insertBefore(logEntry, logDiv.firstChild);
            
            // Keep only last 50 logs
            while (logDiv.children.length > 50) {
                logDiv.removeChild(logDiv.lastChild);
            }
        }
        
        function updateTelemetry(data) {
            document.getElementById('lat').innerHTML = data.lat ? data.lat.toFixed(6) : '---';
            document.getElementById('lng').innerHTML = data.lng ? data.lng.toFixed(6) : '---';
            document.getElementById('head').innerHTML = data.head ? data.head.toFixed(1) + '°' : '---';
            document.getElementById('dist').innerHTML = data.dist ? data.dist.toFixed(1) + 'm' : '---';
            document.getElementById('waterPH').innerHTML = data.waterPH ? data.waterPH.toFixed(2) : '---';
            document.getElementById('waterTDS').innerHTML = data.waterTDS ? data.waterTDS.toFixed(0) + 'ppm' : '---';
            document.getElementById('waterTemp').innerHTML = data.waterTemp ? data.waterTemp.toFixed(1) + '°C' : '---';
            document.getElementById('rssi').innerHTML = data.rssi ? data.rssi + 'dBm' : '---';
            document.getElementById('laserAngle').innerHTML = data.laserAngle ? data.laserAngle + '°' : '---';
            
            // Update laser status display
            const laserStatusSpan = document.getElementById('laserStatusDisplay');
            if (data.laserState === 1) {
                laserStatusSpan.innerHTML = '🔴 LASER ON';
                laserStatusSpan.className = 'laser-status laser-on';
            } else {
                laserStatusSpan.innerHTML = '⚫ LASER OFF';
                laserStatusSpan.className = 'laser-status laser-off';
            }
        }
        
        async function fetchLatestTelemetry() {
            try {
                const response = await fetch('/api/telemetry/latest');
                if (response.ok) {
                    const data = await response.json();
                    updateTelemetry(data);
                    document.getElementById('connectionStatus').innerHTML = '✅ Connected to Pi';
                    document.getElementById('connectionStatus').className = 'status-online';
                }
            } catch (error) {
                document.getElementById('connectionStatus').innerHTML = '❌ Disconnected from Pi';
                document.getElementById('connectionStatus').className = 'status-offline';
            }
        }
        
        async function sendCommand(command) {
            try {
                const response = await fetch('/api/commands/send', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify(command)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    addLog(`✅ Command sent: ${command.cmd}`, 'success');
                    return true;
                } else {
                    addLog(`❌ Failed to send command: ${command.cmd}`, 'error');
                    return false;
                }
            } catch (error) {
                addLog(`❌ Error sending command: ${error}`, 'error');
                return false;
            }
        }
        
        function sendLaserCommand(cmd) {
            if (cmd === 'laser_on') {
                const angle = parseInt(document.getElementById('laserAngleInput').value);
                sendCommand({cmd: cmd, angle: angle});
            } else {
                sendCommand({cmd: cmd});
            }
        }
        
        function sendLaserOnWithAngle() {
            const angle = parseInt(document.getElementById('laserAngleInput').value);
            sendCommand({cmd: 'laser_on', angle: angle});
        }
        
        function sendLaserAngle() {
            const angle = parseInt(document.getElementById('laserAngleInput').value);
            sendCommand({cmd: 'laser_angle', angle: angle});
        }
        
        function sendNavigationCommand(cmd) {
            sendCommand({cmd: cmd});
        }
        
        function sendSystemCommand(cmd) {
            sendCommand({cmd: cmd});
        }
        
        function clearLogs() {
            document.getElementById('logEntries').innerHTML = '<div class="log-entry">Logs cleared</div>';
            addLog('Logs cleared by user');
        }
        
        // Update telemetry every second
        setInterval(fetchLatestTelemetry, 1000);
        
        // Initial fetch
        fetchLatestTelemetry();
        addLog('Web interface loaded and ready');
    </script>
</body>
</html>
'''

# ============================================
# API Routes
# ============================================

@app.route('/')
def index():
    """Root endpoint - serve web interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/telemetry', methods=['POST'])
def receive_telemetry():
    """Receive telemetry from Raspberry Pi"""
    try:
        data = request.json
        data['received_at'] = time.time()
        telemetry_data.append(data)
        
        # Keep only last 1000 entries
        if len(telemetry_data) > 1000:
            telemetry_data.pop(0)
        
        # Print received data
        if data.get('type') == 'laser_status':
            print(f"✓ Laser status: {'ON' if data.get('laser_on') else 'OFF'} (angle: {data.get('angle')}°)")
        elif data.get('type') == 'telem':
            print(f"✓ Telemetry: {data.get('lat', '?')}, {data.get('lng', '?')}, Head: {data.get('head', '?')}°, Laser: {data.get('laserAngle', '?')}°")
        elif data.get('type') == 'heartbeat':
            print(f"✓ Heartbeat received from Pi")
        elif data.get('type') == 'ack':
            print(f"✓ ACK: {data}")
        else:
            print(f"✓ Received: {data.get('type', 'unknown')}")
        
        return jsonify({"status": "ok"}), 200
    except Exception as e:
        print(f"Error receiving telemetry: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/api/commands/pending', methods=['GET'])
def get_pending_commands():
    """Get pending commands for Raspberry Pi"""
    global command_queue
    commands_to_send = command_queue.copy()
    command_queue = []  # Clear after sending
    return jsonify({"commands": commands_to_send}), 200

@app.route('/api/commands/send', methods=['POST'])
def send_command():
    """Send command to Raspberry Pi"""
    try:
        command = request.json
        command['timestamp'] = time.time()
        command_queue.append(command)
        print(f"📨 Command queued: {command}")
        return jsonify({"status": "queued", "command": command}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/telemetry/latest', methods=['GET'])
def get_latest_telemetry():
    """Get latest telemetry data"""
    if telemetry_data:
        return jsonify(telemetry_data[-1]), 200
    return jsonify({"error": "No data"}), 404

@app.route('/api/telemetry/all', methods=['GET'])
def get_all_telemetry():
    """Get all telemetry data"""
    return jsonify(telemetry_data), 200

@app.route('/api/commands/status', methods=['GET'])
def get_command_status():
    """Get queue status"""
    return jsonify({"queued_commands": len(command_queue)}), 200

# ============================================
# Main
# ============================================

def main():
    print("=" * 50)
    print("Laptop Telemetry Server (with Laser Support)")
    print("=" * 50)
    print(f"Server IP: {LAPTOP_IP}")
    print(f"Server Port: {LAPTOP_PORT}")
    print(f"URL: http://{LAPTOP_IP}:{LAPTOP_PORT}")
    print("=" * 50)
    print("\nAvailable endpoints:")
    print("  GET  /                        - Web interface")
    print("  POST /api/telemetry           - Receive telemetry")
    print("  GET  /api/commands/pending    - Get pending commands")
    print("  POST /api/commands/send       - Send command to USV")
    print("  GET  /api/telemetry/latest    - Get latest telemetry")
    print("  GET  /api/telemetry/all       - Get all telemetry")
    print("  GET  /api/commands/status     - Check command queue")
    print("=" * 50)
    print("\n✓ Server starting...")
    print("✓ Open browser and go to: http://" + LAPTOP_IP + ":" + str(LAPTOP_PORT))
    print("✓ Press Ctrl+C to stop\n")
    
    # Run Flask server
    app.run(host='0.0.0.0', port=LAPTOP_PORT, debug=False, threaded=True)

if __name__ == "__main__":
    main()
