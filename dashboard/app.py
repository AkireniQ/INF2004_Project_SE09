#!/usr/bin/env python3
"""Local browser dashboard for the existing Buddy 1 binary MQTT protocol.

Run in Ubuntu/WSL (or Windows Python):
    python3 app.py 192.168.1.10
No firmware or broker WebSocket changes are required.
"""
import argparse
from collections import deque
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import ipaddress
import json
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import urlsplit
import uuid

from mqtt_protocol import command_payload, decode_telemetry, TELEMETRY_TOPICS

HEARTBEAT_TIMEOUT = 15.0
TELEMETRY_TIMEOUT = 2.0
STATIC_DIR = Path(__file__).resolve().parent / "web"


class CommandUnavailable(Exception):
    pass


class Dashboard:
    def __init__(self, broker, broker_port=1883, root="robot/mtk3smp-pico/v2", clock=time.monotonic):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_./-]{0,53}", root):
            raise ValueError("Topic root must be 1..54 ASCII letters, digits, /, _, - or .; no wildcards")
        self.broker, self.broker_port, self.root = broker, broker_port, root
        self.clock = clock
        self.lock = threading.RLock()
        self.client = None
        self.broker_connected = False
        self.monitor_ready = False
        self.last_status = None
        self.heartbeat_at = None
        self.heartbeat_time = None
        self.last_ack = None
        self.last_command = None
        self.error = ""
        self.events = deque(maxlen=40)
        self.sequence = 0
        self.telemetry = {name: None for name in TELEMETRY_TOPICS}
        self.motion_history = deque(maxlen=120)

    def event(self, kind, message):
        # Callers hold the reentrant lock, including callbacks and HTTP threads.
        self.sequence += 1
        self.events.appendleft({"id": self.sequence, "time": self.timestamp(),
                                "kind": kind, "message": message})

    @staticmethod
    def timestamp():
        return datetime.now(timezone.utc).isoformat()

    def start(self, mqtt):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id="buddy1-dashboard-" + uuid.uuid4().hex[:10])
        self.client.on_connect = self.on_connect
        self.client.on_connect_fail = self.on_connect_fail
        self.client.on_disconnect = self.on_disconnect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.connect_async(self.broker, self.broker_port, 60)
        self.client.loop_start()

    def close(self):
        if self.client is not None:
            self.client.disconnect()
            self.client.loop_stop()

    def on_connect(self, client, userdata, flags, reason, properties):
        with self.lock:
            self.broker_connected = not reason.is_failure
            self.monitor_ready = False
            self.heartbeat_at = None
            self.motion_history.clear()
            for sample in self.telemetry.values():
                if sample: sample['session_valid'] = False
            if reason.is_failure:
                self.error = "Broker rejected connection: " + str(reason)
                self.event("error", self.error)
                return
            self.error = ""
            self.event("connection", "Connected to MQTT broker")
            result, _ = client.subscribe([(self.root + "/" + name, 0) for name in ("status", "ack", *TELEMETRY_TOPICS)])
            if result != 0:
                self.error = "Could not subscribe to Pico telemetry topics"
                self.event("error", self.error)

    def on_connect_fail(self, client, userdata):
        with self.lock:
            self.broker_connected = self.monitor_ready = False
            self.heartbeat_at = None
            self.error = "Cannot reach broker. Check its IP, listener and firewall; retrying."
            self.event("error", self.error)

    def on_subscribe(self, client, userdata, mid, reasons, properties):
        with self.lock:
            self.monitor_ready = bool(reasons) and all(not reason.is_failure for reason in reasons)
            if self.monitor_ready:
                self.error = ""
                self.event("connection", "Subscribed to Pico heartbeat, telemetry and acknowledgments")
            else:
                self.error = "Broker rejected a dashboard subscription"
                self.event("error", self.error)

    def on_disconnect(self, client, userdata, flags, reason, properties):
        with self.lock:
            self.broker_connected = self.monitor_ready = False
            self.heartbeat_at = None
            self.motion_history.clear()
            for sample in self.telemetry.values():
                if sample: sample['session_valid'] = False
            self.error = "Broker disconnected; retrying."
            self.event("connection", self.error)

    def on_message(self, client, userdata, message):
        payload = bytes(message.payload)
        with self.lock:
            if message.topic == self.root + "/status":
                if len(payload) != 1 or payload[0] > 31:
                    self.event("error", "Ignored malformed heartbeat")
                    return
                self.last_status = payload[0]
                if message.retain:
                    # Stored online status is not proof that the Pico is alive.
                    self.event("status", "Stored status received; waiting for a live heartbeat")
                else:
                    self.heartbeat_at = self.clock()
                    self.heartbeat_time = self.timestamp()
                    self.event("heartbeat", "Live heartbeat " + payload.hex().upper())
            elif message.topic == self.root + "/ack":
                if len(payload) != 1 or payload[0] not in (0,1,2,3,4,6) or message.retain:
                    self.event("error", "Ignored malformed or retained acknowledgment")
                    return
                self.last_ack = {"command": "reset" if payload[0] & 4 else "start" if payload[0] & 1 else "stop",
                                 "rejected": bool(payload[0] & 2), "time": self.timestamp(),
                                 "hex": payload.hex().upper()}
                label = self.last_ack["command"].upper()
                result = "rejected" if self.last_ack["rejected"] else "accepted"
                self.event("ack", "Pico ACK: " + label + " " + result)
            else:
                prefix = self.root + "/"
                if not message.topic.startswith(prefix): return
                kind = message.topic[len(prefix):]
                if kind not in self.telemetry: return
                if message.retain:
                    self.event("telemetry", "Ignored retained " + kind + " sample; waiting for live data")
                    return
                try:
                    decoded = decode_telemetry(kind, payload)
                except ValueError as exc:
                    self.event("error", "Ignored malformed " + kind + " telemetry: " + str(exc))
                    return
                previous = self.telemetry[kind]
                now = self.clock()
                self.telemetry[kind] = dict(data=decoded, received_at=now,
                                            time=self.timestamp(), session_valid=True)
                if kind == "vehicle" and (previous is None or previous['data'] != decoded):
                    self.event("vehicle", "Mission: " + decoded['state'] +
                               (" · " + ", ".join(decoded['fault_labels']) if decoded['faults'] else ""))
                if kind == "motion":
                    self.motion_history.append(dict(left_rpm=decoded['left_rpm'],
                                                    right_rpm=decoded['right_rpm']))
                    if decoded['faults'] and (previous is None or previous['data']['faults'] != decoded['faults']):
                        self.event("fault", "Motor: " + ", ".join(decoded['fault_labels']))

    def snapshot(self):
        with self.lock:
            age = None if self.heartbeat_at is None else max(0, self.clock() - self.heartbeat_at)
            fresh = age is not None and age <= HEARTBEAT_TIMEOUT
            flags = self.last_status or 0
            ready = self.broker_connected and self.monitor_ready and fresh and flags & 7 == 7
            controller = self.telemetry.get('controller')
            controller_live = bool(controller and controller['session_valid'] and self.broker_connected and
                                   self.clock()-controller['received_at']<=TELEMETRY_TIMEOUT)
            if controller_live and controller['data']['start_block']:
                ready = False
            return {
                "broker": self.broker, "broker_port": self.broker_port, "root": self.root,
                "broker_connected": self.broker_connected, "monitor_ready": self.monitor_ready,
                "error": self.error, "start_allowed": bool(ready),
                "stop_allowed": self.broker_connected,
                "reset_allowed": bool(controller_live and fresh and flags & 7 == 7 and
                                      controller['data']['reset_supported'] and
                                      controller['data']['motor_faults'] and not controller['data']['reset_pending'] and
                                      controller['data']['state'] in ('IDLE','FAULT') and
                                      self.telemetry['motion'] and self.telemetry['motion']['session_valid'] and
                                      self.clock()-self.telemetry['motion']['received_at']<=TELEMETRY_TIMEOUT and
                                      self.telemetry['motion']['data']['mode']==0),
                "pico": {"fresh": fresh, "age": age, "time": self.heartbeat_time,
                         "hex": None if self.last_status is None else f"{flags:02X}",
                         "wifi": bool(flags & 1), "mqtt": bool(flags & 2),
                         "subscribed": bool(flags & 4), "tx_drop": bool(flags & 8),
                         "rx_drop": bool(flags & 16)},
                "telemetry": {
                    name: {"data": sample['data'] if sample else None,
                           "age": max(0, self.clock() - sample['received_at']) if sample else None,
                           "time": sample['time'] if sample else None,
                           "fresh": bool(sample and sample['session_valid'] and self.broker_connected and
                                         self.monitor_ready and self.clock() - sample['received_at'] <= TELEMETRY_TIMEOUT)}
                    for name, sample in self.telemetry.items()
                },
                "motion_history": list(self.motion_history),
                "last_ack": self.last_ack, "last_command": self.last_command,
                "events": list(self.events),
            }

    def send(self, command):
        payload = command_payload(command)  # Only exact START/STOP/RESET; never arbitrary topics/payloads.
        with self.lock:
            state = self.snapshot()
            if self.client is None or not state["broker_connected"]:
                raise CommandUnavailable("Broker is disconnected. No command was queued.")
            if command == "start" and not state["start_allowed"]:
                raise CommandUnavailable("Wait for a live Pico heartbeat with all connection flags ready.")
            if command == "reset" and not state["reset_allowed"]:
                raise CommandUnavailable("Reset requires fresh stopped-motor diagnostics and a latched motor fault.")
            result = self.client.publish(self.root + "/command", payload, qos=0, retain=False)
            if result.rc != 0:
                raise CommandUnavailable("MQTT did not queue the command; check the broker connection.")
            self.last_command = {"command": command, "hex": payload.hex().upper(),
                                 "time": self.timestamp()}
            self.event("command", command.upper() + " queued for broker (" + payload.hex().upper() + ")")
            return self.last_command.copy()


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, dashboard):
        self.dashboard = dashboard
        self.csrf = secrets.token_urlsafe(24)
        super().__init__(address, DashboardHandler)


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # HTTP polling noise is not useful; MQTT events are visible in the dashboard.

    def local_request(self):
        port = self.server.server_port
        hosts = (f"localhost:{port}", f"127.0.0.1:{port}")
        if self.headers.get("Host", "") not in hosts:
            self.json_response(403, {"error": "Use the local dashboard address"})
            return False
        origin = self.headers.get("Origin")
        if origin and origin not in tuple("http://" + host for host in hosts):
            self.json_response(403, {"error": "Cross-origin requests are not allowed"})
            return False
        return True

    def headers_for(self, status, content_type, length):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
        self.end_headers()

    def json_response(self, status, data):
        body = json.dumps(data).encode("utf-8")
        self.headers_for(status, "application/json; charset=utf-8", len(body))
        self.wfile.write(body)

    def do_GET(self):
        if not self.local_request():
            return
        path = urlsplit(self.path).path
        if path == "/api/state":
            state = self.server.dashboard.snapshot()
            state["csrf"] = self.server.csrf
            self.json_response(200, state)
            return
        files = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "text/javascript"),
                 "/styles.css": ("styles.css", "text/css")}
        if path not in files:
            self.json_response(404, {"error": "Not found"})
            return
        name, content_type = files[path]
        body = (STATIC_DIR / name).read_bytes()
        self.headers_for(200, content_type + "; charset=utf-8", len(body))
        self.wfile.write(body)

    def do_POST(self):
        if not self.local_request():
            return
        if urlsplit(self.path).path != "/api/command":
            self.json_response(404, {"error": "Not found"})
            return
        if not hmac.compare_digest(self.headers.get("X-Dashboard-Token", "").encode("utf-8"), self.server.csrf.encode("ascii")):
            self.json_response(403, {"error": "Reload the dashboard before sending commands"})
            return
        if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
            self.json_response(415, {"error": "Expected JSON"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 128:
                raise ValueError("Invalid command size")
            self.connection.settimeout(5)
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict) or set(data) != {"command"} or data["command"] not in ("start", "stop", "reset"):
                raise ValueError("Command must be start, stop or reset")
            result = self.server.dashboard.send(data["command"])
        except CommandUnavailable as exc:
            self.json_response(503, {"error": str(exc)})
        except (ValueError, TypeError, TimeoutError) as exc:
            self.json_response(400, {"error": str(exc)})
        else:
            self.json_response(200, {"queued": True, **result})


def port_number(text):
    value = int(text)
    if not 1 <= value <= 65535:
        raise argparse.ArgumentTypeError("port must be 1..65535")
    return value


def ipv4_address(text):
    try:
        return str(ipaddress.IPv4Address(text))
    except ipaddress.AddressValueError:
        raise argparse.ArgumentTypeError("Enter the actual broker IPv4 address, such as 192.168.1.10") from None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("broker", nargs="?", type=ipv4_address, help="same broker IPv4 address as config/wifi_secrets.h; prompted if omitted")
    parser.add_argument("--broker-port", type=port_number, default=1883)
    parser.add_argument("--root", default="robot/mtk3smp-pico/v2")
    parser.add_argument("--port", type=port_number, default=8080, help="local browser port")
    args = parser.parse_args()
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        parser.exit(1, "Install the dashboard dependency in your venv:\npython3 -m pip install -r requirements.txt\n")
    while args.broker is None:
        try:
            args.broker = ipv4_address(input("Enter your broker IPv4 address (same as the Pico): ").strip())
        except argparse.ArgumentTypeError as exc:
            print(exc)
        except (EOFError, KeyboardInterrupt):
            parser.exit(1, "\nBroker address required. Pass it after app.py.\n")
    try:
        dashboard = Dashboard(args.broker, args.broker_port, args.root)
    except ValueError as exc:
        parser.error(str(exc))
    try:
        server = DashboardServer(("127.0.0.1", args.port), dashboard)
    except OSError as exc:
        parser.exit(1, f"Cannot start local dashboard: {exc}. Try --port 8081.\n")
    try:
        dashboard.start(mqtt)
        print(f"Open http://localhost:{server.server_port} in your browser.", flush=True)
        print(f"Broker: {args.broker}:{args.broker_port} | Topics: {args.root}/...", flush=True)
        print("Send STOP before closing if motion is active. Ctrl+C closes this dashboard.", flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.close()
        server.server_close()


if __name__ == "__main__":
    main()
