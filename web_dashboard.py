import json
import os
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import config

MEDIA_STREAM_URL = "http://localhost:8889/stream"
WEB_PORT = 8080
HTML_PATH = os.path.join(os.path.dirname(__file__), "dashboard.html")


def get_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None


def get_power():
    try:
        volt = subprocess.check_output(
            ["vcgencmd", "measure_volts"], text=True
        ).strip()
        volt_val = float(volt.split("=")[1].replace("V", ""))

        raw = subprocess.check_output(
            ["vcgencmd", "get_throttled"], text=True
        ).strip()

        return {"voltage": volt_val, "throttled": raw.split("=")[1]}
    except Exception:
        return {"voltage": None, "throttled": None}


class DashboardHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/":
            with open(HTML_PATH, "r") as f:
                html = f.read().replace("MEDIA_STREAM_URL", f'"{MEDIA_STREAM_URL}"')
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html.encode("utf-8"))
        elif parsed.path == "/api/status":
            temp = get_temperature()
            power = get_power()
            ball = list(self.server.shared_ball_pos[:])
            data = {
                "temperature": temp,
                "power": power,
                "ball": ball,
                "uart": getattr(self.server, "last_uart", None),
            }
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(data).encode("utf-8"))
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


def web_dashboard(shared_ball_pos):
    server = HTTPServer(("0.0.0.0", WEB_PORT), DashboardHandler)
    server.shared_ball_pos = shared_ball_pos
    server.last_uart = None
    print(f"Dashboard running on http://0.0.0.0:{WEB_PORT}")
    server.serve_forever()
