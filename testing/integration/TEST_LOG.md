# Integration — test log

The motion recovery regression compiles the production steering and controller logic for the host.

```sh
cc -std=c11 -Wall -Wextra -Werror testing/integration/test_motion_recovery.c \
  app_program/subsystem3/line_steering.c app_program/subsystem3/barcode.c \
  app_program/vehicle_controller/vehicle_core.c app_program/subsystem5/s5_avoid.c \
  -o /tmp/test_motion_recovery
/tmp/test_motion_recovery
```

On 21 November 2026 the recovery regression and 23 dashboard tests passed. The Pico W MQTT, offline and isolated motion autotest images linked with the original Subsystem 2 source. A hardware run is recorded separately when performed.

The slower line-following regression adds straight-line noise, a one-sample reflectance spike, tight left and right bends, brief loss and recentering, a broad marking, and a stop after persistent loss during a bend. The motion stack and its pin assignments are unchanged. This test feeds synthetic IR readings into the production steering code.

The wheel matching regression adds a neutral interval before opposite steering, and tests the production motor core against phase-shifted encoder pulses, unequal wheel drag, a stopped inside wheel, bounded turns, and a stalled motor.

The crawl load test interrupts a single wheel's encoder pulses until PWM exceeds the normal crawl cap. It then checks that the assist fades after motion resumes at the same RPM target and that a wheel that remains blocked still faults.

The steering regression also checks that a short opposite edge after a sustained bend produces a smaller correction and that a persistent opposite edge can still start a second bend.

The barcode regression replays alternating edge touches at cruise speed while reading F at all three print sizes. During capture, the wheel difference stays at most 2 RPM and a steering reversal waits for 140 ms of neutral steering. Ordinary tracking uses the team's 40 ms neutral interval. The replay checks steering and decoding with simulated sensor inputs.

```sh
cc -std=c11 -Wall -Wextra -Werror testing/integration/test_motion_crawl.c \
  app_program/subsystem2/motion_crawl.c -o /tmp/test_motion_crawl
/tmp/test_motion_crawl
```

The device-manager lock regression runs two host threads through the production SMP lock implementation with simulated kernel event flags.

```sh
cc -std=gnu11 -Wall -Wextra -Werror -pthread -include stddef.h \
  -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_ -DCNF_SMP=1 -isystem include -isystem config \
  testing/integration/test_fastmlock.c lib/libtk/fastmlock.c -o /tmp/test_fastmlock
/tmp/test_fastmlock
```

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
