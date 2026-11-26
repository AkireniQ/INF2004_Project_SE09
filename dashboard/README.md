# SE09 vehicle dashboard

Browser dashboard for the Pico's existing binary-v2 MQTT firmware. The local Python server connects to your existing TCP MQTT broker; the browser talks only to that server. No broker WebSocket configuration is required.

## Run on your laptop

Use the same LAN broker IPv4 address and topic root as the Pico's ignored `config/wifi_secrets.h`. The root normally is `robot/<MQTT_CLIENT_ID>/v2`. It must match exactly. This dashboard retains the uploaded default `robot/mtk3smp-pico/v2`; if your client ID is `se09-car`, pass `--root robot/se09-car/v2`.

```sh
cd dashboard
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 app.py BROKER_IP --root robot/YOUR_CLIENT_ID/v2
```

On Windows activate with `.venv\Scripts\activate` and use `python` if needed. Replace `BROKER_IP` and `YOUR_CLIENT_ID` with your configuration. If no broker IP is supplied, the app prompts for it. The server prints the local browser address when ready. Defaults: MQTT TCP port 1883, local browser port 8080; override with `--broker-port` and `--port`.

The Pico must run the MQTT firmware (`make NET=lwip MQTT=1`), not the normal non-MQTT build. MQTT firmware boots stopped until START. The dashboard never sends START on startup, reconnect, page reload or automatically after a fault.

## What is displayed

- **Vehicle controller:** mission state and reported subsystem fault flags.
- **Wheels/motion:** measured left and right RPM, propulsion PWM, motor mode, global speed level, recent RPM trend and motor fault labels.
- **Line sensors and barcode:** left and right black-detection flags, the decoded letter and the barcode navigation controls.
- **Hump height:** highest valid completed hump since power-up in millimetres, from live terrain telemetry.
- **Obstacle:** detected, not detected or unavailable, with the last estimated width in millimetres.
- **Connection:** live heartbeat age, Wi-Fi/broker/subscription flags and bridge drop flags.
- **Commands:** binary START, STOP and RESET, Pico acknowledgment and the reason when the controller blocks START.

The outer IR channels steer and the dedicated GP27 ADC channel reads barcodes. The dashboard displays the latest decoded A–Z barcode. The obstacle width is unavailable until there is a usable width estimate. A displayed mission state reports controller intent. Wheel RPM reports measured motion.

## Freshness and commands

Each telemetry channel independently expires after 2 seconds without a valid live sample. The wheel, mission and hump panels label old readings **Stale**. The obstacle status changes to **Unavailable** when detection telemetry expires. Live heartbeats do not freshen old measurements. A reconnect requires new live samples before telemetry becomes fresh again. Retained telemetry and malformed packets never count as live data. No-data and measured zero remain distinct.

Heartbeat expires after 15 seconds. START is available only when the broker/subscriptions and a fresh heartbeat confirm all connection flags. The Pico may still reject START due to startup delay, stale line data, a junction stop or fault. STOP stays available while the local server has a broker connection. No command is retained; the server does not replay commands after reconnect.

The Pico ACK confirms acceptance for processing. Inspect mission state, measured wheel RPM and faults to confirm movement. During an obstacle detour the mission state shows Avoiding, then Recover line before returning to Follow line. A started mission uses local sensor feedback if MQTT goes offline. Dashboard commands require a live connection.

## Assign barcode instructions

With the car stopped, choose four different A–Z letters in **Barcode navigation** and click **Apply mapping**. Wait for the Pico to confirm the active mapping before starting. The controller uses an assigned letter at the next confirmed junction. Straight keeps line tracking active through the crossing. Unassigned letters appear in telemetry without changing navigation. The Pico keeps the mapping in memory until it restarts. The power-up defaults are Left A, Right F, Straight D and U-turn C.

MQTT QoS 0 does not guarantee delivery. The browser reports uncertainty if its HTTP request times out. Faults need investigation before a new START. This is a local dashboard with Host/Origin and command-token checks, bound to loopback; it is not designed for public Internet exposure. Broker TLS/auth configuration is not added by this dashboard or the current Pico MQTT client.

## Protocol

Subscriptions use the configured root and these suffixes:

| Topic | Bytes | Decode |
|---|---:|---|
| `controller` |13 (v3)| bytes 0–6 carry version, state, stop reason, START blocker, readiness/pending flags, motor fault bits and last START rejection. Bytes 7–10 carry the timing fault gap in µs, and bytes 11–12 carry the timing limit in µs. |
| `status` |1| Wi-Fi1, broker2, command subscription4, TX drop8, RX drop16 |
| `ack` |1| START in bit0, RESET in bit2 (neither means STOP); rejected in bit1 |
| `vehicle` |1| state bits0..2, fault mask bits3..7 |
| `motion` |6| mode0:2, level2:3, faults5:4, L RPM9:9, R RPM18:9, L PWM27:7, R PWM34:7 |
| `line` |1| L/R black bits0/2, barcode bits3..7 (0 none, 1–26 A–Z) |
| `barcode_map` |3| Active mapping, with left0:5, right5:5, straight10:5, U-turn15:5, and tag `0xA` at 20:4. Letters use 1–26 for A–Z. |
| `terrain` |2| Session maximum in bits 0–10 (2047 = unavailable); bits 11–13 = 6 identify the maximum format. No measurement is fabricated before the first valid completed hump. |
| `obstacle` |2| width0:11 (2047 means unavailable), detection11:2 (0 unavailable, 1 not detected, 2 detected). Reported on change and refreshed once per second. |

The dashboard can also decode older controller, motion, terrain and obstacle packets. It reads the old `sonar` topic for detection when using older firmware. The updated Pico does not publish that topic.

Bit numbering starts at the least significant bit of byte0. Fields use the firmware's little-endian packing. `offset:width` notation is used above. `mqtt_protocol.py` validates lengths, ranges and unused bits before displaying telemetry. START publishes `0x01`, STOP `0x00`, and stopped-only RESET `0x02` to `/command`, QoS0, non-retained. Strings such as `START` are not valid Pico commands.

To apply a mapping, the dashboard publishes the three-byte mapping packet to `/command`, QoS0 and non-retained. The Pico reports its active mapping on `/barcode_map` once per second. The dashboard confirms the change when that report matches the submitted mapping.

## Tests

State/decoder/HTTP tests (Python standard library):

```sh
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Browser integration requires Mosquitto, Chromium and Playwright in addition to requirements.txt:

```sh
python3 -m pip install playwright
python3 tests/test_browser.py --screenshot /tmp/se09-dashboard.png
```

Set `BUDDY1_MOSQUITTO` or `BUDDY1_BROWSER` if executable paths are not discoverable. The integration test starts its own local broker and simulated-Pico protocol fixture, tests actual HTTP/browser commands and all telemetry topics, then shuts down its processes. The fixture is explicitly not hardware validation. See TEST_RESULTS.md for executed outcomes.

## If START is rejected

Use the current dashboard with the Pico firmware. The Commands panel shows the controller's START blocker when available. Wait at least five seconds after power-up and start on a straight section of tape. Read the blocker before resetting faults and check USB serial for more detail.

## Reset a stopped fault

With current firmware and a live command connection, a stopped motor or controller fault enables **Reset fault**. Press it once and wait for the ACK and IDLE state. After a failed detour, put the car back on a straight section of tape before pressing START separately. If a timing fault returns, capture the displayed **Gap / limit** numbers before clearing it.

The command is binary02; accepted RESET ACK04, rejected RESET ACK06. Existing START/STOP bytes are unchanged. Controller diagnostics version2 has31 bytes and adds reset-pending, latched timing gap and limit; older version1/25-byte diagnostics still display but cannot enable RESET. Raw ADC values and total distance remain unreported.

The hump card displays the maximum valid completed height for the processor session,
not the FIFO's latest completed height. Firmware sends unavailable before the first
measurement (including after reboot); reception freshness is separate from the age
of the hump that established the maximum. Update firmware and dashboard together.
Legacy three-byte reports expose their explicit maximum; legacy two-byte completed
heights are not displayed as a maximum. The completed-event FIFO is unchanged.
