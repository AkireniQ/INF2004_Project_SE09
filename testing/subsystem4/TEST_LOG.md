# Subsystem 4 — test log

**Subsystem:** Monitor (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | Calibration, flat/tilt/ramp/hump sequences, odometry height validity, vibration, and compass-rate estimate | Buddy 4 processing | host | Build and run `testing/buddy4/test_b4_algorithms.c` with the command in its header | All synthetic sequence checks pass; height remains invalid without odometry; no collision event is emitted |
| S4-U04 | Compact event encoding, sample representation, and sensor raw conversion | Buddy 4 data representation | host | Build and run `testing/buddy4/test_b4_foundation.c` with the command in its header | All representation, conversion, and event encoding checks pass |
| S4-U05 | Low-g lift, pronounced tilt, and inverted-orientation detection; single stop request and latched stop state | Buddy 4 safety processing | host | Synthetic IMU samples in `testing/buddy4/test_b4_algorithms.c` | Detection debounces three consecutive 100 Hz samples, requests stop once, and remains latched after the sample returns to normal |
| S4-U06 | False-positive resistance during forward acceleration, a brief low-g transient, and hump traversal | Buddy 4 safety processing | host | Synthetic IMU sequences in `testing/buddy4/test_b4_algorithms.c` | None of the normal-motion sequences requests a stop |
| S4-U07 | Buddy 4 lifecycle starts one production IMU runtime in both build profiles | Buddy 4 runtime | on-target | `subsystem4_start()` starts the shared 100 Hz sampler; `subsystem4_test.c` verifies it is running without starting another sampler | Exactly one acquisition task owns the I2C sensor and the runtime is active |
| S4-U08 | Stop dispatch invokes both motor-stop interfaces only for a stop request | Buddy 4 safety dispatch | host | Mock callbacks in `testing/buddy4/test_b4_algorithms.c` | No callback on ordinary flags; both callbacks exactly once on a stop request; invalid callbacks are rejected |
| S4-HW02 | Lift/overturn stop integration with Buddy 2 and no automatic restart | Buddy 4 to Buddy 2 / vehicle controller | manual | After the shared startup integration is applied, start movement, then lift or overturn the robot; confirm both motors stop and remain stopped after setting it down | Motors stop; no autonomous restart occurs; only an explicit start can resume motion |

### Lift/overturn behavior

The simple safety detector trips after three consecutive fresh accelerometer
samples indicate either near-weightlessness (resultant magnitude at or below
350 mg) or a pronounced orientation change (at least 60 degrees from the
calibrated startup gravity direction while the magnitude remains in the normal
gravity range). The stop-request event and stop-latched status use bits in a
one-byte flag field. A latched trip is not cleared by returning the robot to
its normal orientation.

An accelerometer alone cannot distinguish an upright robot resting on the
floor from one held motionless at the same orientation: both measure gravity
identically. The low-g path detects free-fall-like lifting, and the orientation
path detects a pronounced tilt while lifting or overturning. Verify the actual
pickup motion on hardware; detecting a slow, upright lift with no measurable
acceleration or orientation change would require an additional contact or
proximity sensor.

Buddy 4 calls the existing Buddy 2 `motion_stop()` priority latch, then
`vehicle_request_stop()` to cancel automatic/pending starts. The same 100 Hz
sampler, calibration, processor, stop dispatch, and mutex-protected public
snapshot are now part of `subsystem4_start()` in normal and test builds. Test
builds add only the diagnostic logger; they do not start a second sensor task.

The shared `app_main.c` still calls `subsystem4_init()` and
`subsystem4_start()` only under `APP_TEST`. Production will not launch the
Buddy 4 runtime until the team approves removing those two startup guards.
This Buddy 4 change intentionally does not modify that shared file.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
