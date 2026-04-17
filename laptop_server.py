import json
import os
import time
from threading import Lock
from flask import Flask, jsonify, request, Response

app = Flask(__name__)

# Store latest telemetry from Raspberry Pi
latest_lock = Lock()
latest_telem = {"ok": False, "ts": 0, "raw": None, "data": {}}

def _now_ms() -> int:
    return int(time.time() * 1000)

def _get_latest() -> dict:
    with latest_lock:
        return dict(latest_telem)

@app.route('/api/telemetry', methods=['POST'])
def receive_telemetry():
    """Endpoint for Raspberry Pi to send telemetry data"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({"ok": False, "err": "no_data"}), 400
        
        with latest_lock:
            latest_telem["ok"] = True
            latest_telem["ts"] = _now_ms()
            latest_telem["raw"] = json.dumps(data)
            latest_telem["data"] = data
        
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "err": str(e)}), 500

@app.route('/api/command', methods=['POST'])
def send_command():
    """Endpoint for web UI to send commands to Raspberry Pi"""
    payload = request.get_json(silent=True) or {}
    
    # Store in a queue for Pi to fetch
    with latest_lock:
        if 'pending_commands' not in latest_telem:
            latest_telem['pending_commands'] = []
        latest_telem['pending_commands'].append(payload)
    
    return jsonify({"ok": True})

@app.route('/api/commands/pending', methods=['GET'])
def get_pending_commands():
    """Endpoint for Raspberry Pi to fetch pending commands"""
    with latest_lock:
        commands = latest_telem.get('pending_commands', [])
        latest_telem['pending_commands'] = []
    return jsonify({"commands": commands})

@app.route('/')
def index():
    return Response(INDEX_HTML, mimetype='text/html')

@app.route('/data')
def get_data():
    latest = _get_latest()
    age = 999999
    if latest["ok"] and latest["ts"]:
        age = _now_ms() - int(latest["ts"])
    return jsonify({"age_ms": age, "data": latest["data"]})

INDEX_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>USV Dashboard - Laptop Server</title>
  <style>
    :root { --bg:#0b1220; --card:#121a2b; --text:#e5e7eb; --muted:#94a3b8; --accent:#60a5fa; --ok:#22c55e; --bad:#ef4444; --mono: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; --sans: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Arial; }
    * { box-sizing: border-box; }
    body { margin:0; font-family: var(--sans); background: radial-gradient(900px 600px at 20% 0%, #152447, var(--bg)); color: var(--text); padding: 16px; }
    .wrap { max-width: 980px; margin: 0 auto; display: grid; gap: 14px; }
    .top { display:flex; justify-content: space-between; align-items:center; gap: 10px; padding: 14px 16px; background: rgba(18,26,43,.9); border: 1px solid rgba(148,163,184,.15); border-radius: 14px; }
    .title { font-weight: 900; letter-spacing: .03em; }
    .pill { padding: 6px 10px; border-radius: 999px; background: rgba(96,165,250,.12); border: 1px solid rgba(96,165,250,.25); color: var(--accent); font-family: var(--mono); font-weight: 800; }
    .grid { display:grid; grid-template-columns: 1.2fr 1fr; gap: 14px; }
    .card { background: rgba(18,26,43,.9); border: 1px solid rgba(148,163,184,.15); border-radius: 14px; padding: 14px; display:grid; gap: 12px; }
    .card h3 { margin:0; font-size: 12px; letter-spacing: .12em; text-transform: uppercase; color: var(--muted); }
    .metrics { display:grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    .m { padding: 10px; border-radius: 12px; border: 1px solid rgba(148,163,184,.12); background: rgba(2,6,23,.25); }
    .m .k { font-size: 11px; color: var(--muted); font-weight: 800; }
    .m .v { font-family: var(--mono); font-size: 16px; font-weight: 900; margin-top: 4px; }
    button { border:0; border-radius: 12px; padding: 12px 12px; font-weight: 900; cursor:pointer; }
    .btns { display:grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    .btn-start { background: rgba(34,197,94,.18); border: 1px solid rgba(34,197,94,.35); color: #dcfce7; }
    .btn-stop { background: rgba(239,68,68,.16); border: 1px solid rgba(239,68,68,.35); color: #fee2e2; }
    .btn-cal { background: rgba(96,165,250,.12); border: 1px solid rgba(96,165,250,.30); color: #dbeafe; }
    input[type=range] { width:100%; }
    .thr { display:grid; gap: 10px; }
    .thr .lab { display:flex; justify-content: space-between; font-family: var(--mono); font-weight: 900; }
    @media(max-width: 880px){ .grid{ grid-template-columns: 1fr; } .metrics{ grid-template-columns: 1fr; } .btns{ grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="top">
      <div class="title">USV Dashboard - Laptop Server</div>
      <div class="pill" id="link">STATUS: ---</div>
    </div>
    <div class="grid">
      <div class="card">
        <h3>Telemetry</h3>
        <div class="metrics">
          <div class="m"><div class="k">Lat</div><div class="v" id="lat">---</div></div>
          <div class="m"><div class="k">Lon</div><div class="v" id="lon">---</div></div>
          <div class="m"><div class="k">Heading</div><div class="v" id="head">---</div></div>
          <div class="m"><div class="k">Dist</div><div class="v" id="dist">---</div></div>
          <div class="m"><div class="k">pH</div><div class="v" id="ph">---</div></div>
          <div class="m"><div class="k">TDS</div><div class="v" id="tds">---</div></div>
          <div class="m"><div class="k">Temp</div><div class="v" id="temp">---</div></div>
          <div class="m"><div class="k">RSSI</div><div class="v" id="rssi">---</div></div>
          <div class="m"><div class="k">SNR</div><div class="v" id="snr">---</div></div>
        </div>
      </div>
      <div class="card">
        <h3>Control</h3>
        <div class="btns">
          <button class="btn-start" onclick="sendCmd({cmd:'start'})">Start</button>
          <button class="btn-stop" onclick="sendCmd({cmd:'clear'})">Clear</button>
          <button class="btn-cal" onclick="sendCmd({cmd:'calibrate'})">Calibrate</button>
        </div>
        <div class="thr">
          <div class="lab"><span>Sync</span><span id="vSync">1000</span></div>
          <input type="range" min="1000" max="2000" value="1000" id="sync" oninput="onSync(this.value)">
          <div class="lab"><span>Left</span><span id="vL">1000</span></div>
          <input type="range" min="1000" max="2000" value="1000" id="left" oninput="onLeft(this.value)">
          <div class="lab"><span>Right</span><span id="vR">1000</span></div>
          <input type="range" min="1000" max="2000" value="1000" id="right" oninput="onRight(this.value)">
        </div>
      </div>
    </div>
  </div>
  <script>
    function sendCmd(payload){
      fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload)}).catch(()=>{});
    }
    let lastThrTs = 0;
    function sendThr(l, r){
      const now = Date.now();
      if (now - lastThrTs < 120) return;
      lastThrTs = now;
      sendCmd({cmd:'set_thr', left: Number(l), right: Number(r)});
    }
    function onSync(v){
      document.getElementById('vSync').innerText = v;
      document.getElementById('left').value = v;
      document.getElementById('right').value = v;
      document.getElementById('vL').innerText = v;
      document.getElementById('vR').innerText = v;
      sendThr(v, v);
    }
    function onLeft(v){
      document.getElementById('vL').innerText = v;
      sendThr(v, document.getElementById('right').value);
    }
    function onRight(v){
      document.getElementById('vR').innerText = v;
      sendThr(document.getElementById('left').value, v);
    }
    setInterval(() => {
      fetch('/data').then(r => r.json()).then(d => {
        const age = d.age_ms;
        const ok = age < 5000;
        const link = document.getElementById('link');
        link.innerText = ok ? ('LINK: OK ' + age + 'ms') : ('LINK: LOST ' + age + 'ms');
        link.style.borderColor = ok ? 'rgba(34,197,94,.35)' : 'rgba(239,68,68,.35)';
        link.style.background = ok ? 'rgba(34,197,94,.12)' : 'rgba(239,68,68,.10)';
        link.style.color = ok ? '#dcfce7' : '#fee2e2';
        const t = d.data || {};
        document.getElementById('lat').innerText = (t.lat !== undefined) ? Number(t.lat).toFixed(6) : '---';
        document.getElementById('lon').innerText = (t.lng !== undefined) ? Number(t.lng).toFixed(6) : '---';
        document.getElementById('head').innerText = (t.head !== undefined) ? (Number(t.head).toFixed(1) + '°') : '---';
        document.getElementById('dist').innerText = (t.dist !== undefined) ? (Number(t.dist).toFixed(1) + 'm') : '---';
        document.getElementById('ph').innerText = (t.waterPH !== undefined) ? Number(t.waterPH).toFixed(2) : '---';
        document.getElementById('tds').innerText = (t.waterTDS !== undefined) ? (Math.round(Number(t.waterTDS)) + ' ppm') : '---';
        document.getElementById('temp').innerText = (t.waterTemp !== undefined) ? (Number(t.waterTemp).toFixed(2) + ' °C') : '---';
        document.getElementById('rssi').innerText = (t.rssi !== undefined) ? (Number(t.rssi) + ' dBm') : '---';
        document.getElementById('snr').innerText = (t.snr !== undefined) ? (Number(t.snr).toFixed(1) + ' dB') : '---';
      }).catch(()=>{});
    }, 500);
  </script>
</body>
</html>
"""

if __name__ == '__main__':
    host = os.environ.get('USV_HOST', '0.0.0.0')
    port = int(os.environ.get('USV_PORT', '8000'))
    print(f"=" * 50)
    print(f"Laptop Server Running")
    print(f"=" * 50)
    print(f"Access dashboard at: http://localhost:{port}")
    print(f"Or on network: http://<YOUR_LAPTOP_IP>:{port}")
    print(f"Raspberry Pi should send data to: http://<YOUR_LAPTOP_IP>:{port}/api/telemetry")
    print(f"=" * 50)
    app.run(host=host, port=port, debug=False, threaded=True)
