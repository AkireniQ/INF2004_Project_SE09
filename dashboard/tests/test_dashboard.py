"""Dashboard state/HTTP checks; stdlib only, no broker or paho required."""
import argparse
import http.client
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import Dashboard, DashboardServer, CommandUnavailable, ipv4_address, port_number


class FakeClient:
    def __init__(self):
        self.sent = []
        self.result = 0
        self.subscriptions = []

    def publish(self, topic, payload, qos, retain):
        self.sent.append((topic, payload, qos, retain))
        return SimpleNamespace(rc=self.result)

    def subscribe(self, topics):
        self.subscriptions = topics
        return 0, 1


class DashboardTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.dashboard = Dashboard("192.0.2.1", clock=lambda: self.now)
        self.client = self.dashboard.client = FakeClient()

    def connected(self):
        reason = SimpleNamespace(is_failure=False)
        self.dashboard.on_connect(self.client, None, None, reason, None)
        self.dashboard.on_subscribe(self.client, None, 1, [reason, reason], None)

    def message(self, suffix, payload, retain=False):
        self.dashboard.on_message(self.client, None, SimpleNamespace(
            topic=self.dashboard.root + "/" + suffix, payload=payload, retain=retain))

    def test_offline_blocks_commands(self):
        for command in ("start", "stop"):
            with self.assertRaises(CommandUnavailable):
                self.dashboard.send(command)
        self.assertEqual(self.client.sent, [])

    def test_retained_heartbeat_does_not_enable_start(self):
        self.connected()
        self.message("status", b"\x07", retain=True)
        self.assertFalse(self.dashboard.snapshot()["start_allowed"])
        with self.assertRaises(CommandUnavailable):
            self.dashboard.send("start")
        self.dashboard.send("stop")
        self.assertEqual(self.client.sent[-1][1], b"\x00")

    def test_live_heartbeat_binary_commands_and_staleness(self):
        self.connected()
        self.message("status", b"\x07")
        self.assertTrue(self.dashboard.snapshot()["start_allowed"])
        self.dashboard.send("start")
        self.dashboard.send("stop")
        self.assertEqual(self.client.sent, [("robot/mtk3smp-pico/v2/command", b"\x01", 0, False),
                                            ("robot/mtk3smp-pico/v2/command", b"\x00", 0, False)])
        self.now += 16
        state = self.dashboard.snapshot()
        self.assertFalse(state["start_allowed"])
        self.assertTrue(state["stop_allowed"])

    def test_every_connection_flag_required(self):
        self.connected()
        for value in range(32):
            self.message("status", bytes([value]))
            self.assertEqual(self.dashboard.snapshot()["start_allowed"], value & 7 == 7)

    def test_disconnect_and_reconnect_require_new_live_heartbeat(self):
        self.connected()
        self.message("status", b"\x07")
        self.dashboard.on_disconnect(self.client, None, None, None, None)
        self.assertFalse(self.dashboard.snapshot()["stop_allowed"])
        self.connected()
        self.message("status", b"\x07", retain=True)
        self.assertFalse(self.dashboard.snapshot()["start_allowed"])
        self.assertEqual(self.client.sent, [])  # No auto START on reconnect.

    def test_ack_rejection_and_malformed_packets(self):
        self.message("ack", b"\x01")
        self.assertEqual(self.dashboard.snapshot()["last_ack"]["command"], "start")
        self.message("ack", b"\x03")
        self.assertTrue(self.dashboard.snapshot()["last_ack"]["rejected"])
        for payload in (b"", b"\x01\x00", b"\xff"):
            self.message("ack", payload)
            self.message("status", payload)
        self.message("ack", b"\x00", retain=True)
        self.assertTrue(self.dashboard.snapshot()["last_ack"]["rejected"])
        self.assertIsNone(self.dashboard.snapshot()["pico"]["hex"])

    def test_broker_publish_failure_and_invalid_command(self):
        self.connected()
        self.message("status", b"\x07")
        self.client.result = 4
        with self.assertRaises(CommandUnavailable):
            self.dashboard.send("start")
        self.assertIsNone(self.dashboard.snapshot()["last_command"])
        with self.assertRaises(ValueError):
            self.dashboard.send("forward")

    def test_console_address_and_port_validation(self):
        self.assertEqual(ipv4_address("192.168.1.10"), "192.168.1.10")
        self.assertEqual(port_number("1883"), 1883)
        for value in ("", "YOUR_BROKER_IP", "999.1.1.1"):
            with self.assertRaises(argparse.ArgumentTypeError):
                ipv4_address(value)
        for value in ("0", "65536"):
            with self.assertRaises(argparse.ArgumentTypeError):
                port_number(value)

    def test_topic_validation_and_bounded_log(self):
        for root in ("", "/bad", "bad/#", "bad/+", "x" * 55):
            with self.assertRaises(ValueError):
                Dashboard("broker", root=root)
        for _ in range(60):
            self.message("status", b"\x07")
        self.assertEqual(len(self.dashboard.snapshot()["events"]), 40)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.dashboard = Dashboard("192.0.2.1")
        self.dashboard.client = FakeClient()
        self.dashboard.broker_connected = self.dashboard.monitor_ready = True
        self.dashboard.last_status = 7
        self.dashboard.heartbeat_at = self.dashboard.clock()
        self.server = DashboardServer(("127.0.0.1", 0), self.dashboard)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=3)
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        result = response.status, response.read(), dict(response.getheaders())
        connection.close()
        return result

    def command(self, command, **headers):
        defaults = {"Content-Type": "application/json", "X-Dashboard-Token": self.server.csrf}
        defaults.update(headers)
        return self.request("POST", "/api/command", json.dumps({"command": command}), defaults)

    def test_real_http_assets_and_snapshot(self):
        for path, marker in (("/", b"Robot control"), ("/app.js", b"performSend"),
                             ("/styles.css", b"--teal")):
            status, body, headers = self.request("GET", path)
            self.assertEqual(status, 200)
            self.assertIn(marker, body)
            self.assertIn("Content-Security-Policy", headers)
        status, body, _ = self.request("GET", "/api/state")
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)["start_allowed"])
        self.assertEqual(json.loads(body)["csrf"], self.server.csrf)
        self.assertEqual(self.request("GET", "/../buddy1_dashboard.py")[0], 404)

    def test_http_commands_emit_exact_binary_payloads(self):
        self.assertEqual(self.command("start")[0], 200)
        self.assertEqual(self.command("stop")[0], 200)
        self.assertEqual([entry[1] for entry in self.dashboard.client.sent], [b"\x01", b"\x00"])
        self.assertEqual(self.command("restart")[0], 400)

    def test_origin_host_token_and_content_type(self):
        self.assertEqual(self.command("start", Origin="http://example.com")[0], 403)
        self.assertEqual(self.command("start", Host="evil.example")[0], 403)
        self.assertEqual(self.command("start", **{"X-Dashboard-Token": "wrong"})[0], 403)
        self.assertEqual(self.command("start", **{"Content-Type": "text/plain"})[0], 415)
        self.assertEqual(self.dashboard.client.sent, [])

    def test_offline_rejection_is_visible_to_browser(self):
        self.dashboard.broker_connected = False
        status, body, _ = self.command("start")
        self.assertEqual(status, 503)
        self.assertIn("disconnected", json.loads(body)["error"])


if __name__ == "__main__":
    unittest.main()
