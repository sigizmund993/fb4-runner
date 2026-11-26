"""Serve one read-only FB4 diagnostics HTTP connection supplied by systemd."""

from http.server import BaseHTTPRequestHandler
import json
import os
import re
from select import select
import socket
from time import monotonic
from urllib.parse import urlsplit

from diagnostic_resources import ResourceSampler, pi_temperature
from diagnostic_stream import CAMERA_SOCKET, KEEPALIVE, MAX_PACKET_SIZE, TELEMETRY_SOCKET

DEFAULT_ORIGINS = {
    f"http://{host}:{port}"
    for host in ("localhost", "127.0.0.1", "[::1]")
    for port in (8000, 5173)
}
ALLOWED_ORIGINS = DEFAULT_ORIGINS | set(os.environ.get("FB4_DIAGNOSTIC_ORIGINS", "").split())


def offline_sample(robot_id):
    return json.dumps({
        "version": 1,
        "robot_id": robot_id,
        "telemetry_available": False,
        "uart": {},
        "status": {"rpi_temp": pi_temperature()},
        "ball": {},
    }, separators=(",", ":")).encode()


class DiagnosticsRequest(BaseHTTPRequestHandler):
    def log_message(self, _format, *_args):
        pass

    def _allowed(self):
        origin = self.headers.get("Origin")
        if origin is not None and origin not in ALLOWED_ORIGINS:
            self.send_error(403)
            return False
        return True

    def _respond(self, status, content_type=None):
        self.send_response(status)
        origin = self.headers.get("Origin")
        if origin is not None:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.send_header("Connection", "close")
        self.send_header("Cache-Control", "no-store")
        if content_type is not None:
            self.send_header("Content-Type", content_type)
        self.close_connection = True

    def do_OPTIONS(self):
        if not self._allowed():
            return
        if urlsplit(self.path).path not in ("/telemetry", "/video"):
            self.send_error(404)
            return
        self._respond(204)
        self.send_header("Access-Control-Allow-Methods", "GET")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.end_headers()

    def do_GET(self):
        if not self._allowed():
            return
        path = urlsplit(self.path).path
        if path == "/telemetry":
            with ResourceSampler() as sampler:
                self._stream(TELEMETRY_SOCKET, "text/event-stream", lambda packet: self._event(packet, sampler))
        elif path == "/video":
            self._stream(CAMERA_SOCKET, "multipart/x-mixed-replace; boundary=frame", self._frame)
        else:
            self.send_error(404)

    @staticmethod
    def _connect(source, address):
        try:
            source.connect(address)
        except OSError:
            return False
        return True

    def _stream(self, address, content_type, send_packet):
        robot_id = None
        if address == TELEMETRY_SOCKET:
            host = re.fullmatch(r"fb4-(0[0-9]|1[0-5])", socket.gethostname(), re.IGNORECASE)
            if host is not None:
                robot_id = int(host.group(1))

        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as source:
            source.settimeout(3)
            connected = self._connect(source, address)
            if not connected and robot_id is None:
                self._respond(503)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return

            self._respond(200, content_type)
            self.end_headers()
            last_sample = monotonic()
            while not select([self.connection], [], [], 0)[0]:
                if not connected:
                    send_packet(offline_sample(robot_id))
                    if select([self.connection], [], [], 1)[0]:
                        return
                    connected = self._connect(source, address)
                    continue

                packet = source.recv(MAX_PACKET_SIZE)
                if not packet:
                    break
                if packet != KEEPALIVE:
                    last_sample = monotonic()
                elif robot_id is not None and monotonic() - last_sample >= 2:
                    packet = offline_sample(robot_id)
                send_packet(packet)

    def _event(self, packet, sampler):
        message = b": keepalive\n\n" if packet == KEEPALIVE else b"data: " + packet + b"\n\n"
        self.wfile.write(message)
        metrics = sampler.sample()
        if metrics is not None:
            payload = json.dumps(metrics, separators=(",", ":"), allow_nan=False).encode()
            self.wfile.write(b"event: resources\ndata: " + payload + b"\n\n")

    def _frame(self, packet):
        if packet != KEEPALIVE:
            length = str(len(packet)).encode("ascii")
            header = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + length + b"\r\n\r\n"
            self.wfile.write(header + packet + b"\r\n")


def main():
    with socket.socket(fileno=os.dup(0)) as connection:
        connection.settimeout(3)
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 2)
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_USER_TIMEOUT, 5000)
        try:
            DiagnosticsRequest(connection, connection.getpeername(), None)
        except OSError:
            pass  # A disconnected or stalled reader ends just this connection.


if __name__ == "__main__":
    main()
