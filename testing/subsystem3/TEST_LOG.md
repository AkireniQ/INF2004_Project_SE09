# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Barcode serial log during driving

Open the Pico USB serial port at 115200 baud, put the car on the tape before the white margin, and start a pass with the normal MQTT build. Filter for `[BC]`. The first report marks the start of line following. Reports then summarize each two-second interval's progress, with an immediate report on a decode or when line following pauses. The counts and ADC extrema are cumulative within that stretch of line following.

```text
[BC] following GP27=205..3620 bars=15 window=29 adc_errors=0 sample_gaps=0 max_gap=10 ms decoded=F
```

For one clean pass over a medium Code 39 label, expect 15 detected bars and a 29-element frame. `window` is the largest number of elements held in the decoder during this stretch. `adc_errors` counts failed GP27 reads, and `sample_gaps` counts intervals longer than 35 ms between valid readings. Either can break a frame. If `decoded` shows a letter, the existing `[S3] junction confirmed` log indicates that a turn was triggered at the crossing. The barcode summary is printed by a separate task and works without a broker connection once the car is following.

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
