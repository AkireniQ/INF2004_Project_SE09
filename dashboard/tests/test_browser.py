"""Optional real-broker/browser check. Requires paho, Playwright, Mosquitto and Chromium.
The simulated Pico here is a protocol fixture, not hardware validation.
"""
import argparse
from test_telemetry import motion_packet
import http.client
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import Dashboard, DashboardServer


def wait_until(condition, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(0.05)
    raise AssertionError("Timed out waiting for expected integration behavior")


def main():
    import paho.mqtt.client as mqtt
    from playwright.sync_api import sync_playwright, expect
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--screenshot", default="/tmp/buddy1-dashboard.png")
    args = parser.parse_args()
    broker_binary = os.environ.get("BUDDY1_MOSQUITTO") or shutil.which("mosquitto")
    assert broker_binary, "Install Mosquitto or set BUDDY1_MOSQUITTO to its executable"
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    log = open("/tmp/buddy1-dashboard-broker.log", "w")
    broker = subprocess.Popen([broker_binary, "-p", str(port)], stdout=log, stderr=log)
    server = fixture = dashboard = None
    try:
        def broker_ready():
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                    return True
            except OSError:
                return False
        wait_until(broker_ready)
        commands = []
        subscribed = threading.Event()
        root = "robot/mtk3smp-pico/v2"
        fixture = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="pico-protocol-fixture")
        def on_connect(client, userdata, flags, reason, properties):
            assert not reason.is_failure
            client.subscribe(root + "/command", 0)
        def on_subscribe(client, userdata, mid, reasons, properties):
            subscribed.set()
        def on_message(client, userdata, message):
            assert message.payload in (b"\x00", b"\x01", b"\x02")
            commands.append((bytes(message.payload), message.qos, bool(message.retain)))
            client.publish(root + "/ack", b"\x04" if message.payload==b"\x02" else message.payload, qos=0, retain=False)
            if message.payload==b"\x02":
                cleared=bytearray(31);cleared[0]=2;cleared[4]=1|2|64;cleared[29:31]=(15000).to_bytes(2,"little")
                client.publish(root+"/controller",cleared)
                idle=bytearray(motion_packet());idle[0]&=~3
                client.publish(root+"/motion",idle);client.publish(root+"/vehicle",b"\x00")
        fixture.on_connect, fixture.on_subscribe, fixture.on_message = on_connect, on_subscribe, on_message
        fixture.connect("127.0.0.1", port, 60)
        fixture.loop_start()
        assert subscribed.wait(5)
        fixture.publish(root + "/status", b"\x07", qos=0, retain=True).wait_for_publish(timeout=5)
        # Exercise the actual CLI entry point and its invalid/empty IP prompt.
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            http_port = sock.getsockname()[1]
        cli_log = open("/tmp/buddy1-dashboard-cli.log", "w")
        cli = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parents[1] / "app.py"),
                                "--broker-port", str(port), "--port", str(http_port)],
                               stdin=subprocess.PIPE, stdout=cli_log, stderr=cli_log, text=True)
        try:
            cli.stdin.write("\n127.0.0.1\n")
            cli.stdin.flush()
            def cli_ready():
                try:
                    connection = http.client.HTTPConnection("127.0.0.1", http_port, timeout=0.3)
                    connection.request("GET", "/api/state")
                    response = connection.getresponse()
                    result = response.status == 200 and json.loads(response.read())["broker"] == "127.0.0.1"
                    connection.close()
                    return result
                except OSError:
                    return False
            wait_until(cli_ready)
            assert commands == []
            print("PASS CLI: empty address reprompted; valid IP starts local HTTP dashboard; no startup command")
        finally:
            cli.terminate()
            cli.wait(timeout=5)
            cli.stdin.close()
            cli_log.close()
        clock_offset = [0.0]
        dashboard = Dashboard("127.0.0.1", port, root, clock=lambda: time.monotonic() + clock_offset[0])
        server = DashboardServer(("127.0.0.1", 0), dashboard)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        dashboard.start(mqtt)
        wait_until(lambda: dashboard.snapshot()["monitor_ready"] and dashboard.last_status == 7)
        with sync_playwright() as playwright:
            executable = os.environ.get("BUDDY1_BROWSER") or shutil.which("chromium")
            browser = playwright.chromium.launch(executable_path=executable, headless=True,
                                                 args=["--no-sandbox", "--disable-dev-shm-usage"])
            page = browser.new_page(viewport={"width": 1280, "height": 1100})
            errors = []
            page.on("pageerror", lambda error: errors.append(str(error)))
            page.goto(f"http://127.0.0.1:{server.server_port}")
            expect(page.locator("#status-hex")).to_have_text("07")
            assert page.locator("#start").is_disabled()  # Retained status is not live evidence.
            assert page.locator("#stop").is_enabled()
            assert page.locator(".telemetry-card").count() == 5
            expect(page.locator("#terrain-height")).to_have_text("—")
            expect(page.locator("#obstacle-freshness")).to_have_text("No data")
            assert page.locator(".connection-strip, .control-card, .heartbeat-card").count() == 3
            print("PASS browser: controls, heartbeat, all five telemetry cards and missing-data placeholders render")
            print("PASS browser: retained status does not enable START; connected STOP remains available")
            fixture.publish(root + "/status", b"\x07", qos=0, retain=True).wait_for_publish(timeout=5)
            page.locator("#start").wait_for(state="visible")
            expect(page.locator("#start")).to_be_enabled()
            page.locator("#start").click()
            expect(page.locator("#command-feedback")).to_have_text("Pico ACK: START accepted for processing.")
            page.locator("#stop").click()
            expect(page.locator("#command-feedback")).to_have_text("Pico ACK: STOP accepted for processing.")
            assert commands == [(b"\x01", 0, False), (b"\x00", 0, False)], commands
            print("PASS end-to-end: button -> HTTP -> real Mosquitto -> binary 01/00, QoS 0, non-retained -> fixture ACK -> browser")
            page.locator("#start").click()
            page.locator("#stop").click()
            wait_until(lambda: len(commands) == 4)
            assert commands[-2:] == [(b"\x01", 0, False), (b"\x00", 0, False)]
            expect(page.locator("#command-feedback")).to_have_text("Pico ACK: STOP accepted for processing.")
            print("PASS browser: rapid START then STOP preserves command order")
            # Protocol-correct real-broker telemetry fixtures (not a physical Pico).
            fixture.publish(root + "/vehicle", b"\x01").wait_for_publish(timeout=5)
            for sample in range(24):
                fixture.publish(root + "/motion", motion_packet(left=28 + sample % 5, right=22 + sample % 4)).wait_for_publish(timeout=5)
            fixture.publish(root + "/motion", motion_packet()).wait_for_publish(timeout=5)
            fixture.publish(root + "/line", bytes([1 | (2 << 3)])).wait_for_publish(timeout=5)
            fixture.publish(root + "/terrain", (125 | (2 << 11)).to_bytes(2, "little")).wait_for_publish(timeout=5)
            fixture.publish(root + "/obstacle", (500 | (80 << 12) | (200 << 23) | (300 << 35)).to_bytes(6, "little")).wait_for_publish(timeout=5)
            expect(page.locator("#mission-state")).to_have_text("FOLLOW LINE")
            expect(page.locator("#left-rpm")).to_have_text("30")
            expect(page.locator("#right-rpm")).to_have_text("24")
            expect(page.locator("#left-pwm")).to_have_text("42%")
            expect(page.locator("#left-ir")).to_have_text("Black detected")
            expect(page.locator("#right-ir")).to_have_text("No black")
            expect(page.locator("#barcode")).to_have_text("B")
            expect(page.locator("#terrain-height")).to_have_text("125")
            expect(page.locator("#obstacle-nearest")).to_have_text("500 mm")
            assert page.locator("#left-rpm-path").get_attribute("d")
            print("PASS end-to-end telemetry: all five binary topics decoded over real broker, HTTP and browser")
            # Specific START rejection is surfaced without bypassing Pico guards.
            diag = bytearray(25);diag[0]=1;diag[3]=7;diag[4]=1|2|4|64;diag[24]=7
            diag[6:8]=(900).to_bytes(2,"little");diag[10:12]=(850).to_bytes(2,"little")
            fixture.publish(root + "/controller",diag).wait_for_publish(timeout=5)
            expect(page.locator("#start-blocker")).to_have_text("Junction / line stop latched")
            expect(page.locator("#pico-state")).to_have_text("START blocked by controller")
            assert page.locator("#start").is_disabled() and page.locator("#stop").is_enabled()
            fixture.publish(root + "/ack",b"\x03").wait_for_publish(timeout=5)
            expect(page.locator("#command-feedback")).to_contain_text("Junction / line stop latched")
            diag[3]=0;diag[4]=1|2|64
            fixture.publish(root + "/controller",diag).wait_for_publish(timeout=5)
            expect(page.locator("#start")).to_be_enabled()
            print("PASS diagnostics: specific START blocker, IR values, disabled START and preserved STOP; clearing blocker enables retry")
            fault_diag=bytearray(31);fault_diag[0]=2;fault_diag[1]=5;fault_diag[3]=3;fault_diag[4]=1|2|64;fault_diag[5]=2
            fault_diag[25:29]=(12000).to_bytes(4,"little");fault_diag[29:31]=(15000).to_bytes(2,"little")
            fault_motor=bytearray(motion_packet(2));fault_motor[0]&=~3
            fixture.publish(root+"/controller",fault_diag).wait_for_publish(timeout=5)
            fixture.publish(root+"/motion",fault_motor).wait_for_publish(timeout=5)
            fixture.publish(root+"/vehicle",bytes([5|(1<<3)])).wait_for_publish(timeout=5)
            expect(page.locator("#motor-faults")).to_contain_text("Gap 12.00 ms / limit 15.00 ms")
            expect(page.locator("#reset")).to_be_enabled()
            page.locator("#reset").click()
            expect(page.locator("#command-feedback")).to_have_text("Pico ACK: RESET accepted for processing.")
            expect(page.locator("#mission-state")).to_have_text("IDLE")
            expect(page.locator("#reset")).to_be_disabled()
            assert commands[-1]==(b"\x02",0,False) and len(commands)==5
            print("PASS fault recovery: stopped-only RESET sends02, clears fixture fault, leaves mission IDLE and never sends START")
            fixture.publish(root+"/vehicle",b"\x01").wait_for_publish(timeout=5)
            fixture.publish(root+"/motion",motion_packet()).wait_for_publish(timeout=5)
            # Fresh signal for a representative desktop screenshot.
            fixture.publish(root + "/status", b"\x07", qos=0, retain=True).wait_for_publish(timeout=5)
            page.wait_for_timeout(1100)
            page.screenshot(path=args.screenshot, full_page=True)
            page.set_viewport_size({"width": 390, "height": 844})
            page.wait_for_timeout(200)
            assert page.locator("html").evaluate("el => el.scrollWidth") <= 390, "Mobile horizontal overflow"
            print("PASS browser: mobile layout has no horizontal overflow")
            clock_offset[0] = 3.0
            expect(page.locator("#motion-freshness")).to_contain_text("Stale")
            expect(page.locator("#terrain-freshness")).to_contain_text("Stale")
            expect(page.locator("#mission-description")).to_contain_text("Last report only")
            assert page.locator("#start").is_enabled()  # Heartbeat does not refresh measurements.
            fixture.publish(root + "/motion", motion_packet(4)).wait_for_publish(timeout=5)
            fixture.publish(root + "/vehicle", bytes([5 | (1 << 3)])).wait_for_publish(timeout=5)
            expect(page.locator("#motor-faults")).to_have_text("Wheel stalled")
            expect(page.locator("#mission-state")).to_have_text("FAULT")
            print("PASS browser: independent telemetry expiry and motor/mission fault rendering")
            clock_offset[0] = 20.0
            expect(page.locator("#pico-state")).to_have_text("Heartbeat is stale")
            assert page.locator("#start").is_disabled() and page.locator("#stop").is_enabled()
            print("PASS browser: stale heartbeat disables START and keeps STOP available")
            broker.terminate()
            broker.wait(timeout=5)
            expect(page.locator("#pico-state")).to_have_text("Broker unavailable")
            assert page.locator("#start").is_disabled() and page.locator("#stop").is_disabled()
            assert len(commands) == 5  # No automatic command/replay.
            assert not errors, errors
            print("PASS browser: broker loss disables controls; no auto-command; no JavaScript errors")
            browser.close()
    finally:
        if dashboard:
            dashboard.close()
        if fixture:
            fixture.disconnect()
            fixture.loop_stop()
        if server:
            server.shutdown()
            server.server_close()
        if broker.poll() is None:
            broker.terminate()
            broker.wait(timeout=5)
        log.close()


if __name__ == "__main__":
    main()
