# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Standalone IR hardware test

Build from the repository root with:

```sh
make -C build_make -j8 WIFI=none NET=none MQTT=0 IR_BENCH=1
```

Flash `mtk3pico_smp1_usb_cdc_irbench.uf2` using BOOTSEL. Open the Pico USB serial port at 115200 baud. This mode keeps the motor outputs safe and runs the same ADC driver as the driving firmware. The left line sensor is GP26, the center barcode sensor is GP27, and the right line sensor is GP28.

Hold the sensors at their normal height above white paper. Slide black tape under each sensor in turn for a few seconds. Its raw value should change clearly between white and black. Compare at the same height. Check that scan numbers keep rising and cumulative error counts stay at zero. A value of -1 is a read error. If the scan number freezes while age grows, `adc_gp` shows the last active ADC input while it is being read, or 0 between reads. If the scan number keeps rising but one channel's value barely changes, check that sensor's analog output and ground connection.

Flash the normal MQTT UF2 again to restore dashboard control and driving.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Medium Code 39 regression

The test replays A, F and Z from slides 3, 7 and 11 of the supplied deck. The images are placed at 250 mm wide and the printed bar pattern spans about 243 mm at actual size. Replay covers both reading directions, irregular sample intervals, isolated optical glitches, and discarding incomplete frames.

```sh
cc -std=c11 -Wall -Wextra -Werror testing/subsystem3/test_barcode.c \
  app_program/subsystem3/barcode.c -o /tmp/test_barcode
/tmp/test_barcode
```
