#!/usr/bin/env python3
"""Mock server for AutoLee web UI development.

Serves the dashboard HTML with fake state data and SSE events,
so you can test the UI in a browser without ESP32 hardware.

Usage:
    python tools/mock_server.py
    # then open http://localhost:8080

The mock simulates:
- /api/state          GET   - returns fake device state JSON
- /events             GET   - SSE stream pushing state every 250ms
- /api/toggle_run     POST  - toggles IDLE <-> RUNNING
- /api/profile        POST  - switches speed profile
- /api/endpoint       POST  - adjusts endpoint offsets
- /api/sg_trip        POST  - adjusts SG trip per profile
- /api/work_zone      POST  - adjusts work zone
- /api/batch          POST  - batch target/start/clear
- /api/action         POST  - calibrate, reset_counter, return_home
- /api/current        POST  - adjusts motor current (mA)
- /api/log_clear      POST  - clears log
- /api/ota            POST  - fake OTA (always succeeds)
- /api/wifi           POST  - fake wifi save
- /api/wifi_reset     POST  - fake wifi reset

Counter increments automatically when RUNNING.
"""

import json
import time
import threading
import re
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from urllib.parse import urlparse, parse_qs
from pathlib import Path

# ---------------------------------------------------------------------------
# Simulated device state
# ---------------------------------------------------------------------------
state = {
    "version": "1.6-mock",
    "runState": "IDLE",  # IDLE, RUNNING, STOPPING, CALIBRATING, STALLED, HOMING
    "counter": 42,
    "calibrated": True,
    "rawUp": 0,
    "rawDown": 45200,
    "endpointUp": 0,
    "endpointDown": 44700,
    "upOffset": 0,
    "downOffset": -500,
    "position": 0,
    "sgTrip": 15,
    "workZone": 5500,
    "currentMa": 2500,
    "profileIdx": 1,
    "profiles": [
        {"name": "Slow",   "hz": 15000, "sg": 350},
        {"name": "Normal", "hz": 35000, "sg": 15},
        {"name": "Fast",   "hz": 45000, "sg": 1},
    ],
    "batchTarget": 0,
    "batchCount": 0,
    "batchActive": False,
    "wifiStatus": "Connected",
    "wifiSSID": "MockNetwork",
    "wifiIP": "192.168.1.42",
}

log_lines: list[str] = []
sse_clients: list = []
lock = threading.RLock()


def state_json() -> str:
    with lock:
        p = state["profiles"][state["profileIdx"]]
        return json.dumps({
            "version": state["version"],
            "state": state["runState"],
            "counter": state["counter"],
            "speed": p["hz"],
            "calibrated": state["calibrated"],
            "rawUp": state["rawUp"],
            "rawDown": state["rawDown"],
            "endpointUp": state["endpointUp"],
            "endpointDown": state["endpointDown"],
            "upOffset": state["upOffset"],
            "downOffset": state["downOffset"],
            "position": state["position"],
            "sgTrip": p["sg"],
            "workZone": state["workZone"],
            "currentMa": state["currentMa"],
            "profileIdx": state["profileIdx"],
            "profileName": p["name"],
            "profiles": [{"name": p["name"], "hz": p["hz"], "sg": p["sg"]}
                         for p in state["profiles"]],
            "wifiStatus": state["wifiStatus"],
            "wifiSSID": state["wifiSSID"],
            "wifiIP": state["wifiIP"],
            "batchTarget": state["batchTarget"],
            "batchCount": state["batchCount"],
            "batchActive": state["batchActive"],
        })


def add_log(msg: str):
    with lock:
        log_lines.append(msg)
        print(f"  [LOG] {msg}")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------------------------------------------------------------
# Extract HTML from web.h (between R"rawliteral( and )rawliteral")
# ---------------------------------------------------------------------------
def extract_html() -> str:
    web_h = Path(__file__).resolve().parent.parent / "include" / "web.h"
    content = web_h.read_text()
    m = re.search(r'R"rawliteral\((.*?)\)rawliteral"', content, re.DOTALL)
    if not m:
        raise RuntimeError("Could not find INDEX_HTML in include/web.h")
    return m.group(1)


INDEX_HTML = extract_html()


# ---------------------------------------------------------------------------
# Background ticker: simulate motor motion when RUNNING
# ---------------------------------------------------------------------------
def ticker():
    direction = 1  # 1 = toward down, -1 = toward up
    while True:
        time.sleep(0.25)
        with lock:
            if state["runState"] == "RUNNING":
                # Simulate position moving between endpoints
                speed = state["profiles"][state["profileIdx"]]["hz"]
                step = speed // 16  # rough step per 250ms tick
                state["position"] += direction * step

                if direction == 1 and state["position"] >= state["endpointDown"]:
                    state["position"] = state["endpointDown"]
                    state["counter"] += 1
                    if state["batchActive"]:
                        state["batchCount"] += 1
                        if state["batchCount"] >= state["batchTarget"]:
                            add_log(f"Batch complete: {state['batchCount']}/{state['batchTarget']}")
                            state["batchActive"] = False
                            state["runState"] = "IDLE"
                    direction = -1
                elif direction == -1 and state["position"] <= state["endpointUp"]:
                    state["position"] = state["endpointUp"]
                    direction = 1


t = threading.Thread(target=ticker, daemon=True)
t.start()


# ---------------------------------------------------------------------------
# HTTP Handler
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        # Quieter logging — only show non-SSE requests
        msg = fmt % args
        if "/events" not in msg:
            print(f"  {msg}")

    def _send(self, code, content_type, body):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/" or path == "":
            self._send(200, "text/html", INDEX_HTML)

        elif path == "/api/state":
            self._send(200, "application/json", state_json())

        elif path == "/events":
            # SSE needs an unbounded streaming response. Write raw HTTP/1.0
            # so the browser knows to read until connection close.
            self.wfile.write(b"HTTP/1.0 200 OK\r\n")
            self.wfile.write(b"Content-Type: text/event-stream\r\n")
            self.wfile.write(b"Cache-Control: no-cache\r\n")
            self.wfile.write(b"Access-Control-Allow-Origin: *\r\n")
            self.wfile.write(b"\r\n")
            self.wfile.flush()

            self.wfile.write(f"data: {state_json()}\n\n".encode())
            self.wfile.flush()

            sent_log_idx = len(log_lines)
            try:
                while True:
                    time.sleep(0.25)
                    self.wfile.write(f"data: {state_json()}\n\n".encode())

                    with lock:
                        if len(log_lines) > sent_log_idx:
                            new = log_lines[sent_log_idx:]
                            sent_log_idx = len(log_lines)
                            log_json = json.dumps({"log": new})
                            self.wfile.write(f"event: log\ndata: {log_json}\n\n".encode())

                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return  # skip default close handling

        else:
            self._send(404, "text/plain", "Not found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        def p(key, default=None):
            return params.get(key, [default])[0]

        if path == "/api/toggle_run":
            with lock:
                if state["runState"] == "IDLE":
                    state["runState"] = "RUNNING"
                    add_log("Motor: RUN started")
                elif state["runState"] == "RUNNING":
                    state["runState"] = "IDLE"
                    add_log("Motor: STOPPED")
            self._send(200, "text/plain", "ok")

        elif path == "/api/profile":
            idx = int(p("idx", 0))
            with lock:
                if 0 <= idx < len(state["profiles"]):
                    state["profileIdx"] = idx
                    add_log(f"Profile: {state['profiles'][idx]['name']}")
            self._send(200, "text/plain", "ok")

        elif path == "/api/endpoint":
            which = p("which", "up")
            delta = int(p("delta", 0))
            with lock:
                if which == "up":
                    state["upOffset"] = clamp(state["upOffset"] + delta, -8000, 8000)
                    state["endpointUp"] = state["rawUp"] + state["upOffset"]
                else:
                    state["downOffset"] = clamp(state["downOffset"] + delta, -8000, 8000)
                    state["endpointDown"] = state["rawDown"] + state["downOffset"]
            self._send(200, "text/plain", "ok")

        elif path == "/api/sg_trip":
            profile = int(p("profile", state["profileIdx"]))
            value = p("value")
            delta = p("delta")
            with lock:
                if 0 <= profile < len(state["profiles"]):
                    if value is not None:
                        v = int(value)
                    elif delta is not None:
                        v = state["profiles"][profile]["sg"] + int(delta)
                    else:
                        v = state["profiles"][profile]["sg"]
                    state["profiles"][profile]["sg"] = clamp(v, 0, 500)
            self._send(200, "text/plain", "ok")

        elif path == "/api/current":
            ma = p("ma")
            if ma is not None:
                with lock:
                    state["currentMa"] = clamp(int(ma), 1000, 4500)
                    add_log(f"Current set to {state['currentMa']} mA")
            self._send(200, "text/plain", "ok")

        elif path == "/api/work_zone":
            delta = int(p("delta", 0))
            with lock:
                state["workZone"] = clamp(state["workZone"] + delta, 0, 20000)
            self._send(200, "text/plain", "ok")

        elif path == "/api/batch":
            delta = p("delta")
            action = p("action")
            with lock:
                if delta is not None:
                    v = state["batchTarget"] + int(delta)
                    state["batchTarget"] = clamp(v, 0, 9999)
                if action == "start" and state["batchTarget"] > 0:
                    state["batchCount"] = 0
                    state["batchActive"] = True
                    state["runState"] = "RUNNING"
                    add_log(f"Batch started: target={state['batchTarget']}")
                elif action == "clear":
                    state["batchTarget"] = 0
                    state["batchCount"] = 0
                    state["batchActive"] = False
            self._send(200, "text/plain", "ok")

        elif path == "/api/action":
            action = p("do")
            with lock:
                if action == "calibrate":
                    state["runState"] = "CALIBRATING"
                    add_log("Calibration started...")
                    # Simulate calibration completing after a delay
                    def finish_cal():
                        time.sleep(3)
                        with lock:
                            state["runState"] = "IDLE"
                            state["calibrated"] = True
                            state["rawUp"] = 0
                            state["rawDown"] = 45200
                            state["endpointUp"] = 0
                            state["endpointDown"] = 44700
                            add_log("CAL: up=0 dn=45200 travel=45200")
                    threading.Thread(target=finish_cal, daemon=True).start()

                elif action == "reset_counter":
                    state["counter"] = 0
                    add_log("Counter reset")

                elif action == "return_home":
                    if state["runState"] == "STALLED":
                        state["runState"] = "HOMING"
                        add_log("Creep home: start")
                        def finish_home():
                            time.sleep(2)
                            with lock:
                                state["runState"] = "IDLE"
                                state["position"] = state["endpointUp"]
                                add_log("Creep home: done")
                        threading.Thread(target=finish_home, daemon=True).start()
            self._send(200, "text/plain", "ok")

        elif path == "/api/log_clear":
            with lock:
                log_lines.clear()
            self._send(200, "text/plain", "ok")

        elif path == "/api/ota":
            # Read and discard body
            content_length = int(self.headers.get("Content-Length", 0))
            if content_length > 0:
                self.rfile.read(content_length)
            self._send(200, "text/plain", "OK")

        elif path == "/api/wifi":
            ssid = p("ssid", "")
            add_log(f"WiFi saved: {ssid} (mock — no reboot)")
            self._send(200, "text/plain", "saved")

        elif path == "/api/wifi_reset":
            add_log("WiFi cleared (mock — no reboot)")
            self._send(200, "text/plain", "cleared")

        else:
            self._send(404, "text/plain", "Not found")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


if __name__ == "__main__":
    port = 8080
    server = ThreadedHTTPServer(("0.0.0.0", port), Handler)
    print(f"AutoLee mock server running at http://localhost:{port}")
    print("Press Ctrl+C to stop\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
