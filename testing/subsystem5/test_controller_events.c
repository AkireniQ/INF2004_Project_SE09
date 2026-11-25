/* Host exercise of the production dispatcher with a simulated motion owner. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem4/b4_safety_dispatch.h"
#include "../../app_program/subsystem4/b4_terrain_report.h"
#include "../../app_program/vehicle_controller/vehicle_controller.c"

static uint32_t clock_ms;
static unsigned event_flags, stop_count, command_count, scan_count, clear_count;
static unsigned ranging_wakes;
static bool near_tape;
static unsigned search_progress;
static unsigned connection = 7;
static bool motor_busy;
static S2_STATUS motor;
static S2_REPLY motor_reply;
static uint16_t queued_command;

static B4_PROCESSOR imu;
static bool exercise_imu;
static unsigned rearm_count;
uint8_t b4_safety_restart(uint8_t rearm)
{
    if (!exercise_imu) return 1U;
    if (!rearm) return b4_processor_restart_ready(&imu, clock_ms);
    uint8_t ready = b4_processor_rearm(&imu, clock_ms);
    rearm_count += ready;
    return ready;
}

ID tk_cre_mtx(const T_CMTX *config) { assert(config->mtxatr == TA_INHERIT); return 1; }
ER tk_del_mtx(ID id) { assert(id == 1); return E_OK; }
ER tk_loc_mtx(ID id, TMO time) { assert(id == 1 && time == TMO_FEVR); return E_OK; }
ER tk_unl_mtx(ID id) { assert(id == 1); return E_OK; }
ID tk_cre_flg(const T_CFLG *config) { assert(!config->iflgptn); return 2; }
ER tk_set_flg(ID id, UINT bits) { assert(id == 2); event_flags |= bits; return E_OK; }
ER tk_get_otm(SYSTIM *time) { time->lo = clock_ms; return E_OK; }
INT tm_printf(const UB *format, ...) { (void)format; return 0; }
void s3_log_sampler_state(void) {}
ID app_start_task(FP entry, PRI priority, UW cpu, const char *name)
{
    (void)entry; (void)priority; (void)cpu; (void)name;
    assert(0); /* The test drives the worker directly. */
    return E_OBJ;
}
ER tk_wai_flg(ID id, UINT wanted, UINT mode, UINT *seen, TMO timeout)
{
    (void)id; (void)wanted; (void)mode; (void)seen; (void)timeout;
    assert(0);
    return E_OBJ;
}

unsigned s1_connection_status(void) { return connection; }
uint32_t s2_core_timing_fault_gap_us(void) { return 0; }
bool s2_core_distance_reached(unsigned mm, const S2_STATUS *status)
{ (void)mm; (void)status; return near_tape; }
unsigned s2_core_distance_travelled_mm(const S2_STATUS *status)
{ (void)status; return search_progress; }
bool s2_core_maneuver_finished(unsigned arg, const S2_STATUS *status)
{ (void)arg; return s2_get_bits(status, 0, 2) == 0; }
bool s2_core_arc_finished(bool right, const S2_STATUS *status)
{ (void)right; return s2_get_bits(status, 0, 2) == 0; }
static unsigned pivot_angle;
bool s2_core_pivot_reached(unsigned degrees, const S2_STATUS *status)
{ (void)status; return pivot_angle >= degrees; }
bool motion_get_feedback(S2_STATUS *status, S2_REPLY *reply, bool *busy)
{
    *status = motor;
    if (reply) *reply = motor_reply;
    *busy = motor_busy;
    return true;
}

void motion_stop(void)
{
    ++stop_count;
    queued_command = 0;
    motor_busy = true;
}

bool motion_set_wheel_rpm_tracked(unsigned left, unsigned right, uint16_t *command)
{
    assert(!queued_command && left && right);
    ++command_count;
    queued_command = (uint16_t)(1 + left + right * 128);
    *command = queued_command;
    motor_busy = true;
    return true;
}

bool motion_clear_faults(void) { ++clear_count; return true; }
bool motion_maneuver(unsigned mm, bool right, uint16_t *command)
{
    assert(!queued_command && mm && !right);
    *command = queued_command = (uint16_t)(1u + (mm << 4));
    motor_busy = true;
    return true;
}
bool motion_maneuver_reverse(unsigned mm, uint16_t *command)
{
    assert(!queued_command && mm);
    *command = queued_command = (uint16_t)(S2_MANEUVER | ((S2_MANEUVER_REVERSE_BASE + mm) << 4));
    motor_busy = true;
    return true;
}
bool motion_arc(bool right, uint16_t *command)
{
    assert(!queued_command);
    *command = queued_command = (uint16_t)(S2_ARC | ((unsigned)right << 4));
    motor_busy = true;
    return true;
}
bool motion_crawl_pivot(unsigned degrees, bool right, uint16_t *command)
{
    assert(!queued_command);
    pivot_angle = 0;
    *command = queued_command = (uint16_t)(S2_CRAWL_PIVOT | ((degrees * 2u + right) << 4));
    motor_busy = true;
    return true;
}
void subsystem5_wake(void)
{
    if (pending.scan == SCAN_READY) ++scan_count;
    else ++ranging_wakes;
}

static void motor_acknowledge(unsigned reason)
{
    assert(queued_command);
    motor_reply.command = queued_command;
    queued_command = 0;
    ++motor_reply.sequence;
    motor_reply.reason = reason;
    motor_reply.accepted = reason == S2_REPLY_OK;
    motor_busy = motor_reply.accepted;
    s2_put_bits(&motor, 0, 2, motor_reply.accepted);
    vehicle_notify_feedback();
}

static void motor_stopped(void)
{
    motor_busy = false;
    s2_put_bits(&motor, 0, 2, 0);
    vehicle_notify_feedback();
}

static TMO work(void)
{
    assert(event_flags == VC_WAKE);
    event_flags = 0;
    return vehicle_service_once();
}

static unsigned vehicle_state(void)
{
    VEHICLE_SNAPSHOT snapshot;
    assert(vehicle_get_snapshot(&snapshot));
    return snapshot.state;
}

static void complete_move(void)
{
    motor_acknowledge(S2_REPLY_OK);
    work();
    motor_stopped();
    work();
}

static void settle_step(void)
{
    clock_ms += 260;
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    work();
}

static void imu_samples(int16_t z, unsigned count)
{
    for (unsigned i = 0; i < count; i++) {
        clock_ms += 10;
        B4_IMU_SAMPLE sample = {.timestamp_ms = clock_ms,
            .accel_mg = {0, 0, z}, .flags = B4_IMU_SAMPLE_FLAG_ACCEL_VALID};
        B4_PROCESSOR_OUTPUT output;
        (void)b4_processor_update(&imu, &sample, 0U, 0, &output);
        (void)b4_safety_dispatch_stop(output.safety_flags, motion_stop, vehicle_request_stop);
    }
}

static void test_imu_restart(void)
{
    vehicle_request_stop();
    work();
    motor_stopped();
    work();
    model = (VC_MODEL){0};
    pending = (__typeof__(pending)){0};
    drive = (__typeof__(drive)){0};
    exercise_imu = true;
    assert(b4_processor_init(&imu, NULL) == B4_CALIBRATION_COLLECTING);
    imu_samples(1000, 110);
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(vehicle_request_start());
    work();
    assert(vehicle_state() == VC_FOLLOW_LINE && rearm_count == 1);
    unsigned stops = stop_count;
    imu_samples(0, 3);
    work();
    motor_stopped();
    work();
    assert(vehicle_state() == VC_IDLE && stop_count > stops && imu.stop_latched);
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(!vehicle_request_start() && rearm_count == 1);
    imu_samples(1000, 40);
    assert(imu.stop_latched); /* Setting down alone does not restart. */
    assert(!vehicle_request_start() && imu.stop_latched && rearm_count == 1); /* Stale line. */
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(vehicle_request_start());
    imu_samples(0, 1); /* Hazard after enqueue, before worker acceptance. */
    work();
    assert(vehicle_state() == VC_IDLE && imu.stop_latched && rearm_count == 1);
    imu_samples(1000, 40);
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(vehicle_request_start());
    assert(imu.stop_latched); /* Enqueue does not rearm. */
    work();
    assert(vehicle_state() == VC_FOLLOW_LINE && !imu.stop_latched && rearm_count == 2);
    stops = stop_count;
    imu_samples(-1000, 40);
    work();
    motor_stopped();
    work();
    assert(vehicle_state() == VC_IDLE && imu.stop_latched && stop_count > stops);
    puts("PASS Buddy 4 blocks unsafe/queued restart and stops again after explicit restart");
}

static void junction_sample(unsigned left, unsigned right, unsigned decoded, unsigned elapsed)
{
    clock_ms += elapsed;
    vehicle_submit_line(left, 0, right, 12, 12, FALSE, decoded);
    work();
}

static void begin_junction(unsigned decoded)
{
    model = (VC_MODEL){.state = VC_FOLLOW_LINE};
    pending = (__typeof__(pending)){0};
    drive = (__typeof__(drive)){0};
    motor_stopped();
    junction_sample(0, 0, decoded, 10);
    assert(vehicle_state() == VC_FOLLOW_LINE && queued_command);
    motor_acknowledge(S2_REPLY_OK);
    work();
    for (unsigned i = 0; i < 2; ++i) {
        junction_sample(850, 850, 0, 10);
        assert(vehicle_state() == VC_FOLLOW_LINE);
    }
    junction_sample(850, 850, 0, 10);
    assert(vehicle_state() == VC_TURNING && model.junction.phase == VC_JUNCTION_ADVANCE);
    assert(!queued_command);
    motor_stopped();
    work();
    junction_sample(850, 850, 0, 240);
    assert(queued_command >> 4 == S5_IR_TO_AXLE_MM + S5_TAPE_WIDTH_MM / 2u);
    complete_move();
    assert(model.junction.phase == VC_JUNCTION_PIVOT);
    junction_sample(0, 0, 0, 240);
    assert((queued_command & 15u) == S2_CRAWL_PIVOT);
}

static void test_barcode_junction(void)
{
    begin_junction('F' - 'A' + 1);
    assert(queued_command >> 4 == 2u * (90u + VC_JUNCTION_SEARCH_DEG) + 1u);
    motor_acknowledge(S2_REPLY_OK);
    work();
    pivot_angle = 50;
    junction_sample(0, 850, 0, 10);
    junction_sample(0, 850, 0, 10);
    junction_sample(0, 0, 0, 10);
    junction_sample(0, 0, 0, 10);
    assert(!model.junction.dark_samples); /* Ignore tape before the branch window. */
    pivot_angle = 83;
    junction_sample(0, 850, 0, 10);
    junction_sample(0, 850, 0, 10);
    junction_sample(0, 0, 0, 10);
    assert(model.junction.phase == VC_JUNCTION_PIVOT);
    junction_sample(0, 0, 0, 10);
    assert(model.junction.phase == VC_JUNCTION_SETTLE && motor_busy && !queued_command);
    motor_stopped();
    work();
    junction_sample(0, 0, 0, 240);
    assert(vehicle_state() == VC_FOLLOW_LINE && !queued_command);
    junction_sample(0, 0, 0, 10);
    assert(queued_command && data.barcode == 6u);
    vehicle_request_stop();
    work();

    begin_junction('A' - 'A' + 1);
    assert(queued_command >> 4 == 2u * (90u + VC_JUNCTION_SEARCH_DEG));
    motor_acknowledge(S2_REPLY_OK);
    work();
    pivot_angle = 90u + VC_JUNCTION_SEARCH_DEG;
    motor_stopped();
    work();
    assert(model.junction.phase == VC_JUNCTION_SWEEP);
    junction_sample(0, 0, 0, 240);
    assert(queued_command >> 4 == 4u * VC_JUNCTION_SEARCH_DEG + 1u);
    motor_acknowledge(S2_REPLY_OK);
    work();
    pivot_angle = 2u * VC_JUNCTION_SEARCH_DEG;
    motor_stopped();
    work();
    assert(vehicle_state() == VC_FAULT && model.reason == VC_REASON_BARCODE_LINE &&
           model.reposition_required && !queued_command);

    begin_junction('C' - 'A' + 1);
    assert(queued_command >> 4 == 2u * (180u + VC_JUNCTION_SEARCH_DEG) + 1u);
    motor_acknowledge(S2_REPLY_OK);
    work();
    vehicle_request_stop();
    work();
    assert(vehicle_state() == VC_IDLE && model.reposition_required && !queued_command);
    motor_stopped();
    work();
    puts("PASS barcode junction centres the axle, confirms tape and bounds the search");
}

int main(void)
{
    uint32_t request;
    VEHICLE_SNAPSHOT snapshot;
    assert(vehicle_init() == E_OK);
    vehicle_enable_obstacle_monitor();
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    assert(work() == TMO_FEVR);
    assert(vehicle_get_snapshot(&snapshot) && !snapshot.obstacle_no_echo &&
           snapshot.obstacle_forward_mm == S5_UNKNOWN_RANGE && snapshot.start_block == VC_START_READY);

    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_forward_mm == S5_UNKNOWN_RANGE);
    assert(vehicle_request_start());
    vehicle_request_stop();
    assert(work() == TMO_FEVR && stop_count == 1 && !command_count);
    assert(!vehicle_request_start());
    motor_stopped();
    work();
    assert(vehicle_request_start());
    vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
    assert(work() > 0 && vehicle_state() == VC_FOLLOW_LINE && command_count == 1);
    assert(!scan_count && !vehicle_obstacle_scan_request(&request, NULL));
    assert(vehicle_get_snapshot(&snapshot) && !snapshot.obstacle_no_echo && !snapshot.obstacle_blocked);
    vehicle_submit_line(0, 0, 0, 24, 24, FALSE, 0);
    work();
    assert(command_count == 1);
    motor_acknowledge(S2_REPLY_SETTLING);
    work();
    assert(command_count == 1); /* Wait before retrying a settling rejection. */
    clock_ms += VC_REFRESH_MS;
    vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
    work();
    assert(command_count == 2);
    motor_acknowledge(S2_REPLY_OK);
    work();

    for (unsigned i = 0; i < 3; ++i) {
        clock_ms += 30;
        vehicle_submit_obstacle_ping(s5_classify_echo(S5_ECHO_FALL_TIMEOUT, 0));
        vehicle_submit_line(0, 0, 0, 30, 30, FALSE, 0);
        assert(work() > 0 && vehicle_state() == VC_FOLLOW_LINE);
        assert(stop_count == 1 && scan_count == 0);
        assert(vehicle_get_snapshot(&snapshot) && snapshot.line_valid &&
               snapshot.timestamp_ms == snapshot.line_timestamp_ms &&
               snapshot.obstacle_forward_mm == S5_UNKNOWN_RANGE && !snapshot.obstacle_no_echo);
    }

    vehicle_submit_obstacle_range(S5_FOLLOW_STOP_MM);
    work();
    assert(vehicle_state() == VC_AVOIDING && stop_count == 2 && scan_count == 0);
    assert(!vehicle_obstacle_scan_request(&request, NULL));
    motor_stopped();
    work();
    unsigned scan_limit;
    assert(scan_count == 1 && vehicle_obstacle_scan_request(&request, &scan_limit));
    assert(scan_limit == S5_FOLLOW_CLEAR_MM);
    assert(vehicle_obstacle_scan_current(request));

    connection = 0;
    vehicle_notify_feedback();
    assert(work() > 0 && vehicle_state() == VC_AVOIDING && vehicle_obstacle_scan_current(request));
    vehicle_request_stop();
    assert(work() == TMO_FEVR && vehicle_state() == VC_IDLE);
    assert(!vehicle_obstacle_scan_current(request));
    S5_PROFILE stale = {.nearest_mm = S5_SCAN_STOP_MM};
    vehicle_finish_obstacle_scan(&stale, request);
    assert(!event_flags && command_count == 2);

    /* A cancelled detour can be reset even with an empty motor fault mask. */
    connection = 7;
    vehicle_submit_obstacle_range(600);
    assert(vehicle_request_reset());
    assert(work() > 0 && pending.reset && !vehicle_request_start());
    motor_stopped();
    assert(work() == TMO_FEVR && !pending.reset && !clear_count);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE &&
           snapshot.avoidance_phase == S5_OFF && !snapshot.reset_pending);
    assert(command_count == 2);

    s2_put_bits(&motor, 5, 4, 4);
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && vehicle_state() == VC_FAULT);
    assert(vehicle_request_reset());
    assert(work() > 0 && !clear_count);
    motor_stopped();
    assert(work() > 0 && clear_count == 1 && pending.reset);
    s2_put_bits(&motor, 5, 4, 0);
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && !pending.reset);
    assert(vehicle_state() == VC_IDLE && command_count == 2);

    assert(vehicle_request_reset());
    work();
    clock_ms += VC_RESET_TIMEOUT_MS;
    vehicle_notify_feedback();
    assert(work() == TMO_FEVR && !pending.reset && motor_busy);
    motor_stopped();
    work();
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(606);
    assert(vehicle_request_start());
    work();
    motor_acknowledge(S2_REPLY_OK);
    work();
    vehicle_submit_obstacle_range(208);
    work();
    motor_stopped();
    work();
    assert(vehicle_obstacle_scan_request(&request, NULL));

    S5_PROFILE unresolved = {.nearest_mm = 208, .front_mm = 203, .width_mm = S5_UNKNOWN_WIDTH,
                             .angle_offset_deg = 60, .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                             .left_path_mm = 80, .right_path_mm = S5_UNKNOWN_RANGE};
    uint32_t first_request = request;
    clock_ms += 20;
    vehicle_finish_obstacle_scan(&unresolved, request);
    assert(work() > 0 && vehicle_obstacle_scan_request(&request, NULL) && request != first_request);
    clock_ms += 20;
    vehicle_finish_obstacle_scan(&unresolved, request);
    assert(work() > 0 && vehicle_state() == VC_AVOIDING);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.avoidance_phase == S5_WAIT_PATH &&
           snapshot.avoidance_failure == S5_FAIL_ECHO && snapshot.avoidance_failed_phase == S5_PROFILE_PATH);
    vehicle_submit_obstacle_range(606);
    vehicle_request_stop();
    work();
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE && snapshot.obstacle_profile &&
           snapshot.start_block == VC_START_STOP_PENDING);
    assert(snapshot.near_mm == 208 && snapshot.obstacle_forward_mm == 606 && snapshot.width_mm == S5_UNKNOWN_WIDTH);
    motor_stopped();
    work();
    assert(vehicle_request_reset());
    work();
    motor_stopped();
    work();
    assert(vehicle_get_snapshot(&snapshot) && !snapshot.obstacle_profile &&
           snapshot.avoidance_failure == S5_FAIL_NONE && !snapshot.reset_pending);
    /* START with a close obstacle requests avoidance directly from standstill. */
    vehicle_submit_line(108, 0, 26, 30, 30, FALSE, 0);
    vehicle_submit_obstacle_range(208);
    unsigned before_start = command_count;
    assert(vehicle_request_start());
    assert(work() > 0 && vehicle_state() == VC_AVOIDING && command_count == before_start);
    assert(vehicle_obstacle_scan_request(&request, NULL));
    S5_PROFILE saved = {
        .nearest_mm = 196, .front_mm = 196, .width_mm = 294, .angle_offset_deg = 60,
        .left_path_mm = 208, .right_path_mm = 651, .left_edge_mm = -254, .right_edge_mm = 40,
        .left_edge_valid = true, .right_edge_valid = true,
    };
    unsigned scans_before = scan_count;
    clock_ms += 20;
    vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
    vehicle_finish_obstacle_scan(&saved, request);
    assert(work() > 0 && model.avoidance.phase == S5_TURN_OUT);
    assert(vehicle_obstacle_ranging_paused() && !queued_command);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_profile && !snapshot.obstacle_no_echo);

    settle_step();
    assert(queued_command == (S2_ARC | (1u << 4)));
    motor_acknowledge(S2_REPLY_OK);
    work();
    /* A late in-flight ping is dropped even if it would indicate an obstacle. */
    vehicle_submit_obstacle_range(60);
    vehicle_submit_obstacle_ping(s5_classify_echo(S5_ECHO_FALL_TIMEOUT, 0));
    assert(!event_flags && !range_valid(clock_ms));
    motor_stopped();
    work();

    settle_step();
    assert(model.avoidance.phase == S5_SHIFT_OUT && (queued_command >> 4) == 102);
    complete_move();
    settle_step();
    assert(queued_command == S2_ARC);
    motor_acknowledge(S2_REPLY_OK);
    work();
    unsigned before_ir_gap = stop_count;
    clock_ms += 170;
    vehicle_notify_feedback();
    assert(work() == VC_REFRESH_MS && model.avoidance.phase == S5_ALIGN_OUTSIDE &&
           stop_count == before_ir_gap && motor_busy);
    motor_stopped();
    work();

    settle_step();
    assert(model.avoidance.phase == S5_PASS && (queued_command >> 4) == 196 + 177 + 160 - 225);
    complete_move();
    settle_step();
    assert(model.avoidance.phase == S5_TURN_IN && queued_command == S2_ARC);
    complete_move();

    settle_step();
    assert(model.avoidance.phase == S5_FIND_LINE && (queued_command >> 4) == 200 + S5_SEARCH_EXTENSION_MM);
    motor_acknowledge(S2_REPLY_OK);
    work();
    near_tape = true;
    search_progress = 10;
    for (unsigned i = 0; i < 2; ++i) {
        clock_ms += 10;
        vehicle_submit_line(200, 0, 26, 30, 30, FALSE, 0);
        work();
    }
    assert(model.avoidance.phase == S5_FIND_LINE && motor_busy);

    search_progress = 28;
    unsigned stops_before_rejoin = stop_count;
    for (unsigned i = 0; i < 2; ++i) {
        clock_ms += 10;
        vehicle_submit_line(108, 0, 200, 30, 30, FALSE, 0);
        work();
    }
    assert(model.avoidance.align_at_mm == 102);
    assert(model.avoidance.phase == S5_FIND_LINE && motor_busy && stop_count == stops_before_rejoin);
    search_progress = 101;
    vehicle_notify_feedback();
    work();
    assert(model.avoidance.phase == S5_FIND_LINE && motor_busy && stop_count == stops_before_rejoin);

    search_progress = 102;
    vehicle_notify_feedback();
    work();
    assert(model.avoidance.phase == S5_CENTER_LINE && !queued_command && stop_count == stops_before_rejoin + 1);
    search_progress = 103;
    motor_stopped();
    work();
    settle_step();
    assert(model.avoidance.phase == S5_ALIGN_LINE && !queued_command);
    settle_step();
    assert(queued_command == (S2_ARC | (1u << 4)));
    complete_move();
    assert(model.avoidance.phase == S5_CHECK_LINE && vehicle_obstacle_ranging_paused());

    unsigned wakes_before = ranging_wakes;
    clock_ms += 260;
    vehicle_submit_line(23, 850, 33, 30, 30, FALSE, 0);
    work();
    clock_ms += 10;
    vehicle_submit_line(23, 850, 33, 30, 30, FALSE, 0);
    work();
    assert(model.avoidance.phase == S5_COMPLETE && !vehicle_obstacle_ranging_paused());
    assert(ranging_wakes == wakes_before + 1 && scan_count == scans_before);
    assert(!vehicle_obstacle_scan_request(&request, NULL) && !queued_command);
    clock_ms += 10;
    vehicle_submit_obstacle_ping(s5_classify_echo(S5_ECHO_FALL_TIMEOUT, 0));
    vehicle_submit_line(23, 850, 33, 30, 30, FALSE, 0);
    assert(work() > 0 && vehicle_state() == VC_FOLLOW_LINE && queued_command);
    assert(vehicle_get_snapshot(&snapshot) && snapshot.line_valid &&
           snapshot.timestamp_ms == snapshot.line_timestamp_ms &&
           snapshot.obstacle_forward_mm == S5_UNKNOWN_RANGE && !snapshot.obstacle_no_echo);

    /* STOP while off the tape must also wake forward ranging. */
    vehicle_request_stop();
    work();
    motor_stopped();
    work();
    model = (VC_MODEL){.state = VC_RECOVER_LINE, .avoidance = {
        .phase = S5_FIND_LINE, .phase_at = clock_ms, .side = S5_RIGHT, .search_mm = 168,
    }};
    wakes_before = ranging_wakes;
    settle_step();
    assert(vehicle_obstacle_ranging_paused() && queued_command);
    vehicle_request_stop();
    work();
    assert(!vehicle_obstacle_ranging_paused() && ranging_wakes == wakes_before + 1);
    assert(vehicle_state() == VC_IDLE && model.avoidance.failure == S5_FAIL_STOP);

    vehicle_submit_obstacle_range(500);
    assert(!obstacle_blocked());
    vehicle_submit_obstacle_range(S5_FOLLOW_STOP_MM + 1);
    assert(!obstacle_blocked() && obstacle_range.blocked);
    vehicle_submit_obstacle_range(S5_FOLLOW_STOP_MM);
    assert(obstacle_blocked());
    vehicle_submit_obstacle_range(S5_FOLLOW_CLEAR_MM);
    assert(obstacle_blocked());
    vehicle_submit_obstacle_range(S5_FOLLOW_CLEAR_MM + 1);
    assert(!obstacle_blocked());

    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    vehicle_submit_obstacle_range(S5_OPEN_RANGE);
    motor_stopped();
    work();
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_forward_mm == S5_OPEN_RANGE &&
           !snapshot.obstacle_blocked && snapshot.start_block == VC_START_OBSTACLE);

    /* RESET acknowledges an interrupted off-track bypass without authorizing movement. */
    assert(vehicle_state() == VC_IDLE && !vehicle_request_start());
    assert(vehicle_request_reset());
    work();
    motor_stopped();
    work();
    assert(vehicle_get_snapshot(&snapshot) && snapshot.state == VC_IDLE &&
           snapshot.avoidance_failure == S5_FAIL_NONE && !snapshot.obstacle_no_echo &&
           snapshot.start_block == VC_START_OBSTACLE && !queued_command);
    assert(!vehicle_request_start());
    vehicle_submit_line(23, 850, 33, 30, 30, FALSE, 0);
    work();
    assert(vehicle_state() == VC_IDLE && !queued_command);
    assert(vehicle_request_start());
    work();
    assert(vehicle_state() == VC_FOLLOW_LINE && queued_command && !model.reposition_required);

    vehicle_request_stop();
    work();
    motor_stopped();
    work();
    model = (VC_MODEL){.state = VC_AVOIDING, .avoidance = {
        .phase = S5_WAIT_PATH, .phase_at = clock_ms, .scan_retried = true, .blocked_range_mm = 200,
    }};
    vehicle_submit_obstacle_range(200);
    work();
    scans_before = scan_count;
    vehicle_submit_obstacle_ping(s5_classify_echo(S5_ECHO_FALL_TIMEOUT, 0));
    work();
    assert(obstacle_blocked() && !range_valid(clock_ms));
    assert(vehicle_get_snapshot(&snapshot) && snapshot.obstacle_no_echo && snapshot.line_valid);
    assert(vehicle_request_start());
    work();
    assert(model.avoidance.phase == S5_WAIT_PATH && scan_count == scans_before && !queued_command);
    vehicle_submit_obstacle_range(200);
    work();
    assert(vehicle_request_start());
    work();
    assert(model.avoidance.phase == S5_PROFILE_PATH && scan_count == scans_before + 1 &&
           !queued_command);

    puts("PASS missing echo does not start a scan, close echo stops before scanning, and bypass preserves STOP");
    test_barcode_junction();
    test_imu_restart();
}
