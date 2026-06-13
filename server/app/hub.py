#!/usr/bin/env python3
"""ESP32 Hub: MQTT subscriber + SQLite history + web dashboard"""
import json, sqlite3, time, threading, os
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import paho.mqtt.client as mqtt

DB = os.path.expanduser('~/esp32-hub/data/hub.db')
MQTT_HOST = 'localhost'  # 远程 Mosquitto 服务器
LATEST = {}  # {device_mac: {data}}

# ---------- SQLite ----------
def init_db():
    os.makedirs(os.path.dirname(DB), exist_ok=True)
    db = sqlite3.connect(DB, check_same_thread=False)
    db.execute('''CREATE TABLE IF NOT EXISTS sensors (
        ts INTEGER, device TEXT, presence INTEGER,
        moving INTEGER, static INTEGER, distance INTEGER,
        light INTEGER, heap INTEGER)''')
    db.execute('CREATE INDEX IF NOT EXISTS idx_ts ON sensors(ts)')
    db.execute('CREATE INDEX IF NOT EXISTS idx_dev ON sensors(device)')
    db.commit()
    return db

def insert(db, device, data):
    db.execute('INSERT INTO sensors VALUES(?,?,?,?,?,?,?,?)',
        (int(time.time()), device, data.get('st',0), data.get('mv',0),
         data.get('se',0), data.get('dst',0), data.get('lit',0), data.get('heap',0)))
    db.commit()
    db.execute("DELETE FROM sensors WHERE ts < ?", (int(time.time())-604800,))
    db.commit()

def query(db, device, hours):
    rows = db.execute(
        "SELECT ts,presence,moving,static,distance,light,heap FROM sensors "
        "WHERE device=? AND ts>? ORDER BY ts",
        (device, int(time.time())-hours*3600)).fetchall()
    return [{'ts':r[0],'st':r[1],'mv':r[2],'se':r[3],'dst':r[4],'lit':r[5],'heap':r[6]} for r in rows]

# ---------- MQTT ----------
def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"MQTT connected (rc={reason_code})")
    client.subscribe("esp32/+/data")

def on_message(client, userdata, msg):
    try:
        device = msg.topic.split('/')[1]
        data = json.loads(msg.payload)
        data['_ts'] = time.time()
        LATEST[device] = data
        insert(userdata, device, data)
    except Exception as e:
        print(f"MQTT err: {e}")

def mqtt_loop():
    db = init_db()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.user_data_set(db)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, 1883, 60)
    client.loop_forever()

# ---------- HTTP ----------
class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        u = urlparse(self.path)
        qs = parse_qs(u.query)
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')

        if u.path == '/api/devices':
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            devs = {d: {'st':v.get('st'),'mv':v.get('mv'),'se':v.get('se'),
                        'dst':v.get('dst'),'lit':v.get('lit'),'heap':v.get('heap'),
                        'name':v.get('name',''),'ts':v.get('_ts')} for d,v in LATEST.items()}
            self.wfile.write(json.dumps(devs).encode())

        elif u.path == '/api/history':
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            device = qs.get('device',[''])[0]
            hours = int(qs.get('hours',['1'])[0])
            db = sqlite3.connect(DB)
            rows = query(db, device, hours)
            db.close()
            self.wfile.write(json.dumps(rows).encode())

        elif u.path == '/' or u.path == '/dashboard':
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            with open(os.path.expanduser('~/esp32-hub/dashboard.html'), 'rb') as f:
                self.wfile.write(f.read())

        elif u.path == '/health':
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'status':'ok','devices':len(LATEST)}).encode())

        else:
            self.send_error(404)

if __name__ == '__main__':
    print("Starting ESP32 Hub (MQTT mode)...")
    threading.Thread(target=mqtt_loop, daemon=True).start()
    time.sleep(2)
    print("Hub: http://0.0.0.0:8080")
    HTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
