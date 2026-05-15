from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from mqtt_handler import MQTTClient
import asyncio
import json

app = FastAPI()

# ── State terbaru dari ESP32 ─────────────────────────────────────────────────
latest_data = {
    "fallStatus":    "Waiting...",  # "FALL" | "SAFE" | "DISCONNECTED"
    "bpm":           0,             # float, 0 jika leads off
    "fallProb":      0.0,           # float 0.0–1.0
    "ledStatus":     "OFF",         # "ON" | "OFF"
    "buzzerStatus":  "OFF",         # "ON" | "OFF"
    "buzzerReason":  "NONE",        # "NONE" | "BPM_LOW:52" | "BPM_HIGH:135" | "HRV:95"
    "ecg_raw":       0,             # int 0–4095
    "hrv":           0.0,           # RMSSD ms
    "posture":       "UNKNOWN",     # "STANDING" | "SITTING" | "LYING" | "UNKNOWN"
}

connected_clients: list[WebSocket] = []
_loop: asyncio.AbstractEventLoop | None = None

# ── Broadcast ─────────────────────────────────────────────────────────────────
async def _broadcast(data: dict):
    if not connected_clients:
        return
    payload = json.dumps(data)
    dead = []
    for ws in connected_clients:
        try:
            await ws.send_text(payload)
        except Exception:
            dead.append(ws)
    for ws in dead:
        connected_clients.remove(ws)

# ── MQTT callback ─────────────────────────────────────────────────────────────
def handle_mqtt_message(topic: str, payload: str):
    global latest_data

    if topic == "kel1/pa/fallStatus":
        latest_data["fallStatus"] = payload

    elif topic == "kel1/pa/heartRateStatus":
        try:    latest_data["bpm"] = float(payload)
        except: latest_data["bpm"] = 0

    elif topic == "kel1/pa/fallProb":
        try:    latest_data["fallProb"] = float(payload)
        except: latest_data["fallProb"] = 0.0

    elif topic == "kel1/pa/ledStatus":
        latest_data["ledStatus"] = payload

    elif topic == "kel1/pa/buzzerStatus":
        latest_data["buzzerStatus"] = payload

    elif topic == "kel1/pa/buzzerReason":
        latest_data["buzzerReason"] = payload

    elif topic == "kel1/pa/ecgRaw":
        try:    latest_data["ecg_raw"] = int(payload)
        except: latest_data["ecg_raw"] = 0

    elif topic == "kel1/pa/hrv":
        try:    latest_data["hrv"] = float(payload)
        except: latest_data["hrv"] = 0.0

    elif topic == "kel1/pa/posture":
        latest_data["posture"] = payload

    print(f"[MQTT] {topic} -> {payload}")

    if _loop and not _loop.is_closed():
        _loop.call_soon_threadsafe(
            _loop.create_task,
            _broadcast(dict(latest_data))
        )

# ── Init MQTT ─────────────────────────────────────────────────────────────────
mqtt_service = MQTTClient(
    broker   = "broker.hivemq.com",
    port     = 1883,
    callback = handle_mqtt_message,
)

@app.on_event("startup")
async def startup():
    global _loop
    _loop = asyncio.get_running_loop()
    mqtt_service.start()
    print("[APP] Started — event-driven mode aktif")

@app.get("/")
async def get():
    with open("templates/index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    print(f"[WS] Client connected. Total: {len(connected_clients)}")
    try:
        await websocket.send_json(latest_data)
    except Exception:
        pass
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[WS] Error: {e}")
    finally:
        if websocket in connected_clients:
            connected_clients.remove(websocket)
        print(f"[WS] Client disconnected. Total: {len(connected_clients)}")