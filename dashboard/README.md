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
- **Wheels/motion:** measured left/right RPM; propulsion PWM; actual motor mode; global speed level; recent 120-sample RPM trend; raw motor-fault labels; bounded-move progress counts.
- **Line/barcode:** left/right black-detection flags, centre flag, reported barcode letter.
- **Terrain:** reported height in millimetres and terrain phase.
- **Obstacle:** nearest distance, width, left/right clearance in millimetres.
- **Connection:** live heartbeat age, Wi-Fi/broker/subscription flags and bridge drop flags.
- **Commands/events:** exact binary START/STOP, Pico acknowledgment and a bounded connection/mission/fault event list.

All values come from received MQTT payloads. The current two-sensor Pico configuration leaves the centre flag unset; an unset flag does not prove a centre sensor is installed or reading white. Barcode decoding is currently disabled. Terrain and obstacle producers are not connected in the normal firmware, so those cards correctly stay **No data** until real producers submit valid measurements. They do not display invented zeros.

The matching diagnostics firmware additionally publishes normalized IR darkness, target RPM, controller stop reason and telemetry-drop count on `/controller`. Older firmware does not publish these fields. Raw ADC values, tilt, turn rate and total distance remain unavailable. Wheel progress is counts for bounded moves, not cumulative distance. The global speed level is a legacy selection, not the per-wheel fine-RPM target. A displayed mission state reports controller intent; actual wheel RPM reports measured motion.

## Freshness and commands

Each telemetry channel independently expires after 2 seconds without a valid live sample. Its last received value remains visible but faded and labelled **Stale**. Live heartbeats do not freshen old measurements. A reconnect requires new live samples before telemetry becomes fresh again. Retained telemetry and malformed packets never count as live data. No-data and measured zero remain distinct.

Heartbeat expires after 15 seconds. START is available only when the broker/subscriptions and a fresh heartbeat confirm all connection flags. The Pico may still reject START due to startup delay, stale line data, a junction stop or fault. STOP stays available while the local server has a broker connection. No command is retained; the server does not replay commands after reconnect.

The Pico ACK confirms acceptance for processing, not physical movement. Inspect mission state, wheel RPM and faults after commanding START. The first-junction halt remains intentional; barcode turns, obstacle bypass and complete-track navigation remain unimplemented in the current Pico firmware.

MQTT QoS 0 does not guarantee delivery. The browser reports uncertainty if its HTTP request times out. Faults need investigation before a new START. This is a local dashboard with Host/Origin and command-token checks, bound to loopback; it is not designed for public Internet exposure. Broker TLS/auth configuration is not added by this dashboard or the current Pico MQTT client.

## Protocol

Subscriptions use the configured root and these suffixes:

| Topic | Bytes | Decode |
|---|---:|---|
| `controller` |25 (v1) / 31 (v2)| versioned startup guard, readiness, normalized IR, target RPM, line age and drop count; see VEHICLE_CONTROLLER.md |
| `status` |1| Wi-Fi1, broker2, command subscription4, TX drop8, RX drop16 |
| `ack` |1| START in bit0, RESET in bit2 (neither means STOP); rejected in bit1 |
| `vehicle` |1| state bits0..2, fault mask bits3..7 |
| `motion` |9| S2_STATUS packed bits: mode0:2, level2:3, faults5:4, L RPM9:9, R RPM18:9, L PWM27:7, R PWM34:7, L progress41:13, R progress54:13 |
| `line` |1| L/C/R black bits0/1/2, barcode bits3..5 |
| `terrain` |2| height0:11, phase11:3 |
| `obstacle` |6| nearest0:12, width12:11, left23:12, right35:12 |

Bit numbering starts at the least significant bit of byte0. Fields use the firmware's little-endian packing. `offset:width` notation is used above. `mqtt_protocol.py` validates lengths, ranges and unused bits before displaying telemetry. START publishes `0x01`, STOP `0x00`, and stopped-only RESET `0x02` to `/command`, QoS0, non-retained. Strings such as `START` are not valid Pico commands.

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

Replace both the dashboard and Pico firmware with matching current versions. A heartbeat confirms network connectivity, not that the motors and sensors are ready. The Vehicle controller panel reports the actual START blocker. Wait at least five seconds after power-up and start on a straight section, not a crossing. Read the named blocker before resetting faults. If it says diagnostics unavailable, the Pico is running older firmware or the topic root does not match. Send the blocker text and IR/target values for diagnosis; a generic one-byte rejected ACK alone cannot identify the cause.

## Reset a motor fault

With matching timing/reset firmware, a stopped motor fault enables **Reset fault**. Press it once and wait for the ACK and IDLE state, then press START separately. It never resumes automatically. If a timing fault returns, capture the displayed **Gap / limit** numbers before clearing it. A timing gap over the configured limit still stops the car.

The command is binary02; accepted RESET ACK04, rejected RESET ACK06. Existing START/STOP bytes are unchanged. Controller diagnostics version2 has31 bytes and adds reset-pending, latched timing gap and limit; older version1/25-byte diagnostics still display but cannot enable RESET. Raw ADC values and total distance remain unreported.
