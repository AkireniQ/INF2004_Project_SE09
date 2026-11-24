# Subsystem 2 — test log

**Subsystem:** Producer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | sample values stay within 0..100 | | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | free slots and free blocks within pool size | IF-02 | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-M01 | Isolated motion recovery test | Motor control and command transport | on-target auto | Secure both drive wheels off the ground and boot the `MOTION_AUTOTEST=1` image | The 16 queue/priority checks pass, both encoders report movement, and the sequence ends with zero motor faults |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-21 | S2-M01 | `fe356af` | SE09 operator and Codex | PASS | [USB console log](logs/2026-11-21-motion-recovery.txt) | Both wheels responded to the forward and ±45° turn targets. The test ended with both motor faults and queue failures at zero. Floor distance, angle and line tracking were outside the raised-wheel test. |
