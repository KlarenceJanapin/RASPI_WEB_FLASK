#!/usr/bin/env python3
"""
Laptop Server for USV Control
Includes Laser ON/OFF buttons that send commands to the Pi
"""

from flask import Flask, render_template_string, jsonify, request
from flask_cors import CORS
import json
import time
from datetime import datetime
from collections import deque
import threading

app = Flask(__name__)
CORS(app)  # Allow cross-origin requests

# Store latest telemetry data
latest_telemetry = {
    "lat": 0, "lng": 0, "head": 0, "tbrng": 0, "relBearing": 0,
    "dist": 0, "xte": 0, "wp": 0, "direction": "STRAIGHT",
    "intensity": 0, "thrL": 1500, "thrR": 1500, "manual": 0,
    "gpsValid": 0, "gpsSats": 0, "waterTemp": 0, "waterPH": 0,
    "waterTDS": 0, "rssi": -100, "snr": 0, "laserAngle": 0,
    "last_update": 0
}

# Queue for commands pending to send to Pi
command_queue = deque(maxlen=100)

# Telemetry history for graphs (last 100 points)
telemetry_history = deque(maxlen=100)

# HTML Template with Laser Controls
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>USV Control Station - Laser Control</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #eee;
            padding: 20px;
        }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
        }
        
        h1 {
            margin-bottom: 20px;
            font-size: 28px;
            display: flex;
            align-items: center;
            gap: 15px;
        }
        
        .status-badge {
            font-size: 14px;
            padding: 5px 12px;
            border-radius: 20px;
            background: #2c2c3e;
        }
        
        .status-badge.online {
            background: #00b894;
            color: white;
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .card {
            background: rgba(30, 30, 50, 0.9);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 20px;
            border: 1px solid rgba(255,255,255,0.1);
        }
        
        .card-title {
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 15px;
            color: #00b894;
            border-left: 3px solid #00b894;
            padding-left: 10px;
        }
        
        /* Laser Control Styles */
        .laser-control {
            display: flex;
            gap: 20px;
            justify-content: center;
            padding: 20px;
        }
        
        .laser-btn {
            padding: 20px 40px;
            font-size: 24px;
            font-weight: bold;
            border: none;
            border-radius: 50px;
            cursor: pointer;
            transition: all 0.3s ease;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }
        
        .laser-on {
            background: linear-gradient(135deg, #ff6b6b, #ee5a24);
            color: white;
            box-shadow: 0 0 20px rgba(238, 90, 36, 0.5);
        }
        
        .laser-on:hover {
            transform: scale(1.05);
            box-shadow: 0 0 30px rgba(238, 90, 36, 0.8);
        }
        
        .laser-off {
            background: linear-gradient(135deg, #4a4a5a, #3a3a4a);
            color: #aaa;
        }
        
        .laser-off:hover {
            transform: scale(1.05);
            background: linear-gradient(135deg, #5a5a6a, #4a4a5a);
        }
        
        .laser-status {
            text-align: center;
            margin-top: 15px;
            padding: 10px;
            border-radius: 10px;
            font-size: 18px;
            font-weight: bold;
        }
        
        .laser-status.active {
            background: rgba(255, 107, 107, 0.2);
            color: #ff6b6b;
        }
        
        .laser-status.inactive {
            background: rgba(100, 100, 120, 0.2);
            color: #aaa;
        }
        
        .servo-angle {
            text-align: center;
            margin-top: 10px;
            font-size: 14px;
            color: #888;
        }
        
        /* Metrics Grid */
        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
        }
        
        .metric {
            background: rgba(0,0,0,0.3);
            padding: 10px;
            border-radius: 10px;
            text-align: center;
        }
        
        .metric-label {
            font-size: 12px;
            color: #888;
            margin-bottom: 5px;
        }
        
        .metric-value {
            font-size: 20px;
            font-weight: bold;
            color: #00b894;
        }
        
        /* Map */
        #map {
            height: 400px;
            border-radius: 10px;
            margin-top: 10px;
        }
        
        /* Chart */
        canvas {
            max-height: 200px;
        }
        
        /* Responsive */
        @media (max-width: 768px) {
            .grid {
                grid-template-columns: 1fr;
            }
            .laser-btn {
                padding: 15px 30px;
                font-size: 18px;
            }
        }
        
        .full-width {
            grid-column: span 2;
        }
        
        @media (max-width: 768px) {
            .full-width {
                grid-column: span 1;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>
            🚤 USV Control Station
            <span id="connectionStatus" class="status-badge">Connecting...</span>
        </h1>
        
        <div class="grid">
            <!-- Laser Control Card -->
            <div class="card">
                <div class="card-title">🔴 Laser Control (Servo)</div>
                <div class="laser-control">
                    <button class="laser-btn laser-on" onclick="sendLaserCommand('on')">
                        🔴 LASER ON
                        <small>Servo → 20°</small>
                    </button>
                    <button class="laser-btn laser-off" onclick="sendLaserCommand('off')">
                        ⚫ LASER OFF
                        <small>Servo → 0°</small>
                    </button>
                </div>
                <div id="laserStatus" class="laser-status inactive">
                    🔘 Laser Status: OFF
                </div>
                <div id="servoAngle" class="servo-angle">
                    Servo Angle: 0°
                </div>
            </div>
            
            <!-- Telemetry Card -->
            <div class="card">
                <div class="card-title">📡 Live Telemetry</div>
                <div class="metrics-grid">
                    <div class="metric">
                        <div class="metric-label">Latitude</div>
                        <div class="metric-value" id="lat">---</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Longitude</div>
                        <div class="metric-value" id="lng">---</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Heading</div>
                        <div class="metric-value" id="head">---°</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Distance to WP</div>
                        <div class="metric-value" id="dist">---m</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Water Temp</div>
                        <div class="metric-value" id="waterTemp">---°C</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Water pH</div>
                        <div class="metric-value" id="waterPH">---</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">Water TDS</div>
                        <div class="metric-value" id="waterTDS">---ppm</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">LoRa RSSI</div>
                        <div class="metric-value" id="rssi">---dBm</div>
                    </div>
                    <div class="metric">
                        <div class="metric-label">GPS Sats</div>
                        <div class="metric-value" id="gpsSats">---</div>
                    </div>
                </div>
            </div>
        </div>
        
        <!-- Map Card -->
        <div class="card full-width">
            <div class="card-title">🗺️ USV Position</div>
            <div id="map"></div>
        </div>
        
        <!-- Water Quality Chart -->
        <div class="card full-width">
            <div class="card-title">📊 Water Quality History</div>
            <canvas id="qualityChart"></canvas>
        </div>
    </div>
    
    <script>
        // Initialize map
        var map = L.map('map').setView([14.5915, 121.0965], 15);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
        
        var usvMarker = L.circleMarker([0, 0], {
            radius: 8,
            color: '#ff6b6b',
            fillColor: '#ff6b6b',
            fillOpacity: 0.8,
            weight: 2
        }).addTo(map);
        
        var pathLine = L.polyline([], {color: '#00b894', weight: 3}).addTo(map);
        var positionHistory = [];
        
        // Chart setup
        var ctx = document.getElementById('qualityChart').getContext('2d');
        var qualityChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Temperature (°C)',
                        data: [],
                        borderColor: '#ff6b6b',
                        backgroundColor: 'rgba(255, 107, 107, 0.1)',
                        fill: true,
                        yAxisID: 'y'
                    },
                    {
                        label: 'pH',
                        data: [],
                        borderColor: '#00b894',
                        backgroundColor: 'rgba(0, 184, 148, 0.1)',
                        fill: true,
                        yAxisID: 'y'
                    },
                    {
                        label: 'TDS (ppm)',
                        data: [],
                        borderColor: '#fdcb6e',
                        backgroundColor: 'rgba(253, 203, 110, 0.1)',
                        fill: true,
                        yAxisID: 'y1'
                    }
                ]
            },
            options: {
                responsive: true,
                interaction: { mode: 'index', intersect: false },
                plugins: { legend: { position: 'top' } },
                scales: {
                    y: { title: { display: true, text: 'Temp (°C) / pH' } },
                    y1: { position: 'right', title: { display: true, text: 'TDS (ppm)' } }
                }
            }
        });
        
        // Send laser command to server
        async function sendLaserCommand(state) {
            const command = state === 'on' ? 'laser_on' : 'laser_off';
            
            try {
                const response = await fetch('/api/laser', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ command: command })
                });
                
                if (response.ok) {
                    const data = await response.json();
                    console.log('Laser command sent:', data);
                    
                    // Update UI immediately
                    if (state === 'on') {
                        document.getElementById('laserStatus').innerHTML = '🔴 Laser Status: ON';
                        document.getElementById('laserStatus').className = 'laser-status active';
                        document.getElementById('servoAngle').innerHTML = 'Servo Angle: 20°';
                    } else {
                        document.getElementById('laserStatus').innerHTML = '⚫ Laser Status: OFF';
                        document.getElementById('laserStatus').className = 'laser-status inactive';
                        document.getElementById('servoAngle').innerHTML = 'Servo Angle: 0°';
                    }
                }
            } catch (error) {
                console.error('Failed to send laser command:', error);
            }
        }
        
        // Fetch telemetry data
        async function fetchTelemetry() {
            try {
                const response = await fetch('/api/telemetry/latest');
                const data = await response.json();
                
                // Update metrics
                document.getElementById('lat').innerHTML = data.lat?.toFixed(6) || '---';
                document.getElementById('lng').innerHTML = data.lng?.toFixed(6) || '---';
                document.getElementById('head').innerHTML = (data.head?.toFixed(1) || '---') + '°';
                document.getElementById('dist').innerHTML = (data.dist?.toFixed(1) || '---') + 'm';
                document.getElementById('waterTemp').innerHTML = (data.waterTemp?.toFixed(2) || '---') + '°C';
                document.getElementById('waterPH').innerHTML = data.waterPH?.toFixed(2) || '---';
                document.getElementById('waterTDS').innerHTML = (data.waterTDS?.toFixed(0) || '---') + 'ppm';
                document.getElementById('rssi').innerHTML = (data.rssi || '---') + 'dBm';
                document.getElementById('gpsSats').innerHTML = data.gpsSats || '---';
                
                // Update laser status from telemetry
                if (data.laserAngle !== undefined) {
                    if (data.laserAngle >= 20) {
                        document.getElementById('laserStatus').innerHTML = '🔴 Laser Status: ON';
                        document.getElementById('laserStatus').className = 'laser-status active';
                    } else {
                        document.getElementById('laserStatus').innerHTML = '⚫ Laser Status: OFF';
                        document.getElementById('laserStatus').className = 'laser-status inactive';
                    }
                    document.getElementById('servoAngle').innerHTML = `Servo Angle: ${data.laserAngle}°`;
                }
                
                // Update map
                if (data.lat && data.lng && data.lat !== 0 && data.lng !== 0) {
                    usvMarker.setLatLng([data.lat, data.lng]);
                    map.setView([data.lat, data.lng], map.getZoom());
                    
                    // Add to path history
                    positionHistory.push([data.lat, data.lng]);
                    if (positionHistory.length > 100) positionHistory.shift();
                    pathLine.setLatLngs(positionHistory);
                }
                
                // Update chart
                if (data.waterTemp && data.waterPH && data.waterTDS) {
                    const timestamp = new Date().toLocaleTimeString();
                    qualityChart.data.labels.push(timestamp);
                    qualityChart.data.datasets[0].data.push(data.waterTemp);
                    qualityChart.data.datasets[1].data.push(data.waterPH);
                    qualityChart.data.datasets[2].data.push(data.waterTDS);
                    
                    if (qualityChart.data.labels.length > 50) {
                        qualityChart.data.labels.shift();
                        qualityChart.data.datasets.forEach(ds => ds.data.shift());
                    }
                    qualityChart.update();
                }
                
                // Update connection status
                document.getElementById('connectionStatus').innerHTML = '🟢 Online';
                document.getElementById('connectionStatus').className = 'status-badge online';
                
            } catch (error) {
                console.error('Failed to fetch telemetry:', error);
                document.getElementById('connectionStatus').innerHTML = '🔴 Offline';
                document.getElementById('connectionStatus').className = 'status-badge';
            }
        }
        
        // Poll telemetry every 500ms
        setInterval(fetchTelemetry, 500);
        fetchTelemetry();
    </script>
</body>
</html>
"""

# ============================================
# API Routes
# ============================================

@app.route('/')
def index():
    """Serve the main dashboard"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/telemetry', methods=['POST'])
def receive_telemetry():
    """Receive telemetry from Raspberry Pi"""
    global latest_telemetry, telemetry_history
    
    data = request.json
    if data:
        data['last_update'] = time.time()
        latest_telemetry.update(data)
        telemetry_history.append(data)
        
        # Print laser status changes
        if 'laserAngle' in data:
            print(f"[TELEMETRY] Laser angle: {data['laserAngle']}°")
        
        return jsonify({"status": "ok"}), 200
    return jsonify({"status": "error"}), 400

@app.route('/api/telemetry/latest', methods=['GET'])
def get_latest_telemetry():
    """Get the latest telemetry data"""
    return jsonify(latest_telemetry)

@app.route('/api/telemetry/history', methods=['GET'])
def get_telemetry_history():
    """Get telemetry history"""
    return jsonify(list(telemetry_history))

@app.route('/api/laser', methods=['POST'])
def laser_control():
    """Send laser command to Raspberry Pi"""
    data = request.json
    command = data.get('command')
    
    if command not in ['laser_on', 'laser_off']:
        return jsonify({"error": "Invalid command"}), 400
    
    # Add to command queue for Pi to pick up
    command_queue.append({
        "cmd": command,
        "timestamp": time.time()
    })
    
    print(f"[LASER] Command queued: {command}")
    
    return jsonify({
        "status": "queued",
        "command": command
    }), 200

@app.route('/api/commands/pending', methods=['GET'])
def get_pending_commands():
    """Return pending commands for Raspberry Pi to fetch"""
    commands = list(command_queue)
    command_queue.clear()  # Clear after sending
    return jsonify({"commands": commands})

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get system status"""
    return jsonify({
        "status": "running",
        "last_telemetry_age": time.time() - latest_telemetry.get('last_update', time.time()),
        "pending_commands": len(command_queue)
    })

# ============================================
# Main
# ============================================
if __name__ == '__main__':
    print("=" * 60)
    print("🚤 USV Laptop Server (with Laser Control)")
    print("=" * 60)
    print(f"📡 Server running on: http://0.0.0.0:8000")
    print(f"🔴 Laser ON  → Servo 20°")
    print(f"⚫ Laser OFF → Servo 0°")
    print("=" * 60)
    print("Make sure to:")
    print("  1. Update LAPTOP_IP in pi_telemetry_sender.py")
    print("  2. Connect Pi to this laptop via WiFi/Ethernet")
    print("=" * 60)
    
    app.run(host='0.0.0.0', port=8000, debug=True, threaded=True)
