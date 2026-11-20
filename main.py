"""
main.py - Lightweight Python Web Server for RAPID Simulator

Run with:
    python main.py

No external pip packages required (uses Python's built-in standard library).
Automatically opens the 3D RobotStudio-style interface in your default browser.
"""

import json
import os
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer

from kinematics import forward_kinematics, inverse_kinematics, HOME_JOINTS, KinematicsError
from rapid_parser import simulate_rapid, RapidSyntaxError

HOST = "127.0.0.1"
PORT = 8080
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


class RapidSimulatorHandler(BaseHTTPRequestHandler):
    """HTTP request handler serving the 3D frontend and Python simulation API."""

    def _send_json(self, status_code: int, payload: dict):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        return json.loads(raw)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            index_path = os.path.join(STATIC_DIR, "index.html")
            if not os.path.exists(index_path):
                self.send_error(404, "index.html not found in static/ directory")
                return
            with open(index_path, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)
            return

        if self.path == "/api/home":
            fk = forward_kinematics(HOME_JOINTS)
            self._send_json(200, {"joints": HOME_JOINTS, "tcp": fk["tcp"]})
            return

        self.send_error(404, "Not Found")

    def do_POST(self):
        try:
            data = self._read_json()
        except Exception as exc:
            self._send_json(400, {"error": f"Invalid JSON payload: {exc}"})
            return

        if self.path == "/api/simulate":
            code = data.get("code", "")
            try:
                result = simulate_rapid(code, fps=30)
                self._send_json(200, result)
            except RapidSyntaxError as exc:
                self._send_json(400, {"error": str(exc), "line": exc.line_number})
            except Exception as exc:
                self._send_json(500, {"error": f"Simulation error: {exc}"})
            return

        if self.path == "/api/fk":
            joints = data.get("joints", HOME_JOINTS)
            try:
                fk = forward_kinematics([float(j) for j in joints])
                self._send_json(200, fk)
            except Exception as exc:
                self._send_json(400, {"error": str(exc)})
            return

        if self.path == "/api/ik":
            try:
                x = float(data["x"])
                y = float(data["y"])
                z = float(data["z"])
                joints = inverse_kinematics(x, y, z)
                self._send_json(200, {"joints": joints, "tcp": [x, y, z]})
            except (KeyError, ValueError, KinematicsError) as exc:
                self._send_json(400, {"error": str(exc)})
            return

        self.send_error(404, "Unknown API endpoint")

    def log_message(self, format, *args):
        # Keep terminal output clean
        pass


def open_browser(url: str):
    webbrowser.open(url)


def main():
    url = f"http://{HOST}:{PORT}"
    server = HTTPServer((HOST, PORT), RapidSimulatorHandler)
    print("=" * 56)
    print("  ABB RAPID 3D Simulator (Python Backend + WebGL 3D)")
    print(f"  Running at: {url}")
    print("  Press Ctrl+C to stop the server.")
    print("=" * 56)

    threading.Timer(0.6, open_browser, args=(url,)).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down RAPID Simulator...")
        server.server_close()


if __name__ == "__main__":
    main()
