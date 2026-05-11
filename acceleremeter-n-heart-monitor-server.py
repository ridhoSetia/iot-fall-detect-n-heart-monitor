import asyncio
import json
import serial
import serial.tools.list_ports
import websockets

# ========== SESUAIKAN INI ==========
SERIAL_PORT     = "/dev/ttyUSB0"      # Windows: "COM3" | Linux/Mac: "/dev/ttyUSB0"
BAUD_RATE       = 115200
WS_HOST         = "localhost"
WS_PORT         = 8765
RECONNECT_DELAY = 3
DEBUG           = True        # <-- ganti False setelah berhasil
# ====================================

clients = set()

def list_ports():
    ports = serial.tools.list_ports.comports()
    if ports:
        print("[info] Port tersedia:")
        for p in ports:
            print(f"       {p.device} — {p.description}")
    else:
        print("[info] Tidak ada port serial ditemukan.")

def parse_line(line: str):
    """
    Format: counter\tax\tay\taz\tgx\tgy\tgz\ttemp\tecg\tbpm\tleads_off
    Minimal 8 kolom (backward compat jika sensor ECG belum pasang)
    """
    line = line.strip()
    if not line or not line[0].isdigit():
        return None

    for sep in ['\t', ',', ' ']:
        parts = [p.strip() for p in line.split(sep) if p.strip()]
        if len(parts) >= 8:
            try:
                data = {
                    "counter":   int(float(parts[0])),
                    "ax":        float(parts[1]),
                    "ay":        float(parts[2]),
                    "az":        float(parts[3]),
                    "gx":        float(parts[4]),
                    "gy":        float(parts[5]),
                    "gz":        float(parts[6]),
                    "temp":      float(parts[7]),
                    "ecg":       int(float(parts[8]))  if len(parts) > 8  else 0,
                    "bpm":       float(parts[9])       if len(parts) > 9  else 0.0,
                    "lo_p":      int(float(parts[10])) if len(parts) > 10 else 1,
                    "lo_n":      int(float(parts[11])) if len(parts) > 11 else 1,
                    "leads_off": int(float(parts[12])) if len(parts) > 12 else 1,
                }
                return data
            except (ValueError, IndexError):
                continue
    return None

async def broadcast(msg: str):
    if not clients:
        return
    dead = set()
    for c in clients:
        try:
            await c.send(msg)
        except Exception:
            dead.add(c)
    clients.difference_update(dead)

async def serial_reader():
    list_ports()
    line_count = 0
    while True:
        ser = None
        try:
            print(f"[serial] Mencoba buka {SERIAL_PORT} @ {BAUD_RATE}...")
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
            print(f"[serial] Terhubung! Membaca data sensor...")
            ser.reset_input_buffer()

            while True:
                raw = await asyncio.get_event_loop().run_in_executor(None, ser.readline)
                if not raw:
                    if not ser.is_open:
                        raise serial.SerialException("Port tertutup")
                    continue

                line = raw.decode("utf-8", errors="ignore")
                line_count += 1

                if DEBUG and line_count <= 15:
                    print(f"[DEBUG {line_count:02d}] {repr(line)}")

                data = parse_line(line)
                if data:
                    if DEBUG and line_count <= 15:
                        print(f"[PARSED] counter={data['counter']} bpm={data['bpm']} ecg={data['ecg']}")
                    await broadcast(json.dumps(data))

        except serial.SerialException as e:
            print(f"[serial] Error: {e}")
            if ser:
                try: ser.close()
                except: pass
            await broadcast(json.dumps({"error": "sensor_disconnected"}))
            print(f"[serial] Reconnect dalam {RECONNECT_DELAY} detik...")
            await asyncio.sleep(RECONNECT_DELAY)

        except Exception as e:
            print(f"[serial] Error tak terduga: {e}")
            if ser:
                try: ser.close()
                except: pass
            await asyncio.sleep(RECONNECT_DELAY)

async def ws_handler(websocket):
    clients.add(websocket)
    print(f"[ws] Client terhubung: {websocket.remote_address} | total: {len(clients)}")
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"[ws] Client terputus | sisa: {len(clients)}")

async def main():
    print(f"[server] WebSocket ws://{WS_HOST}:{WS_PORT}")
    print(f"[server] Format: counter ax ay az gx gy gz temp ecg bpm leads_off")
    print("-" * 55)
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await serial_reader()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[server] Dihentikan.")