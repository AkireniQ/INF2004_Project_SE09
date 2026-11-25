# Subsystem 5 — test log

**Subsystem:** <name>  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | example: runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | | | manual | 1. … 2. … | |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## SE09 obstacle avoidance

Host checks cover echo classification, coarse-to-fine profiles, clearance decisions, and the complete rounded bypass in each direction. The dispatcher checks stop-before-scan, drops echoes during execution, and verifies that ranging resumes when the line is confirmed or STOP interrupts the bypass. On-car measurements remain pending.

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_scan.c \
  app_program/subsystem5/s5_profile.c -o /tmp/test_s5_scan
/tmp/test_s5_scan

cc -std=c11 -Wall -Wextra -Werror testing/subsystem5/test_avoidance.c \
  app_program/subsystem5/s5_avoid.c app_program/vehicle_controller/vehicle_core.c \
  -o /tmp/test_s5_avoidance
/tmp/test_s5_avoidance

cc -std=gnu11 -Wall -Wextra -Werror -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ \
  -DCNF_SMP=1 -DTM_MQTT=1 -isystem include -isystem config \
  testing/subsystem5/test_controller_events.c app_program/subsystem5/s5_avoid.c \
  app_program/vehicle_controller/vehicle_core.c \
  app_program/subsystem1/s1_wire.c \
  app_program/subsystem4/b4_{imu,event,calibration,processor,safety_dispatch,terrain_report}.c \
  -o /tmp/test_s5_controller
/tmp/test_s5_controller

cc -std=gnu11 -Wall -Wextra -Werror testing/subsystem5/test_arc_motion.c \
  app_program/subsystem2/motion_crawl.c -o /tmp/test_s5_arc
/tmp/test_s5_arc
```

The controller regression also exercises the production Buddy 4 processor and
stop dispatcher: an unsafe START is rejected, a hazard between enqueue and
worker acceptance cancels START, another controller blocker preserves the
latch, and an accepted explicit restart allows a second hazard to stop again.
The RTOS and motor owner are simulated; this is not physical stop validation.

### Stationary sonar bench

```sh
make -C build_make -j8 SMP=1 WIFI=none NET=none MQTT=0 S5_BENCH=1
```

Flash `mtk3pico_smp1_usb_cdc_s5bench.uf2` and open the USB serial console. The bench leaves the drive outputs off and exercises GP12 (servo), GP16 (TRIG), and GP17 (ECHO through the divider). Facing in the direction the car drives, the transmitter is on the left and the receiver is on the right when the sensor is centred. Verify that the 30° command points right, 90° points forward, and 150° points left. If the mounted servo moves the other way, copy `config/s5_servo_calibration.example.h` to `config/s5_servo_calibration.h`, set `S5_SERVO_REVERSED` to 1, and rebuild. Use the bench to confirm the actual bearings before a track run.

Place a flat box about 200 mm ahead. Confirm the forward echo, five coarse bearings, fine edge samples, nearest distance, width estimate and chosen side. Repeat with the left and the right side obstructed in turn. Check that an empty forward sector reports open or returns a measured background beyond the clearance threshold. No rising edge, a stuck echo pin, and malformed pulses remain unavailable.

Repeat with empty sides and with stationary objects beside the box. At an unresolved side bearing, the final two pings wait 200 ms between attempts and allow up to 100 ms to capture a complete echo. If neither 45° reading qualifies a route, the car can take a fresh 60° reading on a candidate side while stopped. Check the `outward clearance` serial line. The reading needs two usable echoes and enough projected room for the chassis. A near 45° echo that blocks the departure still rejects that side. Verify a right turn when both routes are comparable and a left turn when only the left side fits.

### Track acceptance

Repeat a run on clear tape with no sonar target. An unavailable echo should leave line following active and the servo at 90°. Present a box within 230 mm and verify that the wheels stop and the servo sweeps only if the centered check confirms a close target. If a confirmed close target's echo drops out, the car should hold at rest while checking forward. After a successful detour, line following should resume when the IR sensors confirm the tape, including when forward echoes are unavailable.

Use the MQTT build with a single stationary box, a clear bypass lane, and STOP available. Start on a straight section, with the box ahead and room on one side. Verify that both wheels stop before the sweep, that the car commits only after a scan identifies a usable route, and that the outer IR sensors find the original tape before the dashboard returns to Follow line. Repeat with room on the other side and with both sides blocked before the scan. The firmware uses the measured 130 mm from the outer IR sensors to the axle and their 30 mm left-to-right spacing. Recheck `S5_IR_TO_AXLE_MM` and `S5_OUTER_IR_SPACING_MM` if the sensors are remounted.

Start about 300 mm from a flat box on straight tape. The normal scan starts within 230 mm. With a known 200 mm box centred on the tape, check the USB log for the signed edge offsets and sideways target. With measured edges at ±100 mm, the sideways target is 235 mm. Move the same box left and then right to verify that the two edge offsets and chosen route change. The planner assumes a 160 mm outside chassis width including its tyres and adds a 55 mm gap. Measure the assembled car at its widest point and adjust `S5_CHASSIS_WIDTH_MM` to match.

If both edges reach the sweep endpoints, the car can back up once on verified straight tape. The second scan includes the distance backed up, so it can still measure the same box. Confirm that the car stays stopped if the second scan still cannot find a route. The motor-off bench startup shows the commanded angles and pulse widths. Check that the mounted sensor points roughly 60° right at the 30° command and roughly 60° left at 150°.

The default depth estimate is 55% of the box width, with a 130 mm fallback when width is unavailable. For a confirmed 200 mm width and a 230 mm forward gap, the forward target is 500 mm, including `S5_PASS_BODY_ALLOWANCE_MM` (160 mm). The logged straight pass should equal that target minus the forward progress credited to the departure. Confirm that the servo returns to 90° before the wheels move and that sonar stays paused until the tape is found again. Check that the rear is past the far edge before the car turns inward. For deeper boxes, adjust `S5_PASS_DEPTH_PERCENT` or `S5_PASS_UNCERTAIN_DEPTH_MM` to match the course obstacle. The body allowance should cover the distance from the ultrasonic sensor to the rear plus a clearance margin. If the planned distance exceeds `S5_MAX_PASS_MM` (720 mm), the car should hold before departing.

The log should show two 60° rolling turns on departure and another two on return. Straight bypass legs target the 12 RPM line-follow speed. The two wheel targets on each arc average 12 RPM at the axle midpoint. Verify that both outer IR sensors are on the approach side of the tape when the straight search begins. They should record two crossings at the same IR threshold used for line following. If the search ends without a usable pair, the stop log shows the encoder distance, the edge sample counts and the peak IR readings.

After both crossings, the car keeps rolling straight to the encoder position for the final arc. If the search reaches its cap first, the car completes the remaining distance in a separate bounded move. The USB log shows both crossing distances and the target used for the final arc. The alignment calculation assumes an 18 mm stripe. After the final arc, verify that the car straddles the tape with both outer sensors over white. Verify that a missing forward echo allows line following to resume once the tape is confirmed, and that STOP and the pickup stop can interrupt the preset route.

After a confirmed two-sensor crossing and the final arc, put one outer sensor on a tape edge and confirm the handoff still succeeds. The normal line follower uses that edge to correct the position as it moves.

The controller uses IR freshness during line following, tape search and confirmation, and a reverse along the tape. A delayed IR update during an encoder-only arc or straight pass does not interrupt that move. If IR is unavailable when the car reaches tape search or confirmation, the controller waits at rest for up to one second. Loss of IR during a moving tape search still stops the car.
