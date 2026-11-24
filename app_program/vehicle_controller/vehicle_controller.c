#include "vehicle_controller.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/motion_core.h"
#include "../subsystem2/motion_config.h"
#include "../subsystem2/motion_transport.h"
#include "../subsystem1/s1_comm.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem3/line_steering.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem4/subsystem4.h"
#ifndef VC_AUTO_START
#if TM_MQTT
#define VC_AUTO_START 0
#else
#define VC_AUTO_START 1
#endif
#endif
#define VC_REFRESH_MS 100u
#define VC_ACTION_TIMEOUT_MS 250u
#define VC_RESET_TIMEOUT_MS 2500u
#define VC_ECHO_GRACE_MS 180u
#define VC_RETRACE_MIN_MS 1800u
#define VC_RETRACE_LONG_MS 3600u
#define S5_LINE_DARK 420
#define S5_CENTER_TAPE_DARK 250
#define S5_WIDE_MARKING 750
#define VC_WAKE 1u

enum { SCAN_OFF, SCAN_READY, SCAN_ACTIVE };
static ID guard, work_event;
static VEHICLE_SNAPSHOT data;
static VC_MODEL model;
static struct {
    unsigned start:1, stop:1, line_stop:1, auto_start:1, reset:1;
    unsigned obstacle_monitor:1, profile_valid:1, scan:2, follow_blocked:1;
    unsigned last_echo:3, echo_grace_used:1;
} pending;
static uint32_t reset_started, reset_refreshed;
static S5_RANGE_GATE obstacle_range;
static uint32_t obstacle_range_at, full_profile_at, scan_request_id;
static S5_PROFILE full_profile;
static struct { unsigned left:10, center:10, right:10; } tape_search_peak;
static struct {
    uint32_t sequence, issued_at, refreshed;
    uint16_t command;
    unsigned left_rpm:7, right_rpm:7, moving:1, velocity:1, retry:1, failed:1;
} drive;

static bool line_hit(void)
{
    S5_PHASE phase = model.avoidance.phase;
    unsigned darkness = phase == S5_FIND_LINE || phase == S5_CHECK_LINE ? S3_EDGE_DARKNESS : S5_LINE_DARK;
    return data.l >= darkness || data.r >= darkness;
}
static bool range_valid(uint32_t now)
{
    return pending.obstacle_monitor && s5_range_available(obstacle_range.range_mm) &&
           now - obstacle_range_at <= S5_RANGE_MAX_AGE_MS;
}
static bool obstacle_blocked(void)
{
    return pending.obstacle_monitor && pending.follow_blocked;
}
static uint32_t now_ms(void) { SYSTIM t;tk_get_otm(&t);return t.lo; }
static BOOL lock(void) { return guard>0 && tk_loc_mtx(guard,TMO_FEVR)>=E_OK; }
static void unlock(void) { (void)tk_unl_mtx(guard); }
void vehicle_notify_feedback(void)
{
    ID event = __atomic_load_n(&work_event, __ATOMIC_ACQUIRE);
    if (event > 0) (void)tk_set_flg(event, VC_WAKE);
}

ER vehicle_init(void)
{
    T_CMTX mutex = {.mtxatr = TA_INHERIT};
    guard = tk_cre_mtx(&mutex);
    if (guard < E_OK) return guard;
    T_CFLG event = {0};
    ID id = tk_cre_flg(&event);
    if (id < E_OK) {
        (void)tk_del_mtx(guard);
        guard = 0;
        return id;
    }
    pending.auto_start = VC_AUTO_START;
    __atomic_store_n(&work_event, id, __ATOMIC_RELEASE);
    return E_OK;
}
void vehicle_submit_line(INT l,INT c,INT r,INT left,INT right,BOOL stop,unsigned barcode)
{
    if(l<0||l>1000||c<0||c>1000||r<0||r>1000||left<0||left>126||right<0||right>126||
       (left&1)||(right&1)||barcode>4){vehicle_invalidate_line();return;}
    if(!lock())return;
    uint32_t now=now_ms();
    data.l=l;data.c=c;data.r=r;data.left_rpm=left;data.right_rpm=right;
    data.barcode=barcode;data.line_stop=stop;data.line_valid=TRUE;data.line_timestamp_ms=now;
    /* A stop recommendation must survive a newer non-stop sensor sample. */
    if (stop && (model.state == VC_FOLLOW_LINE || pending.start)) pending.line_stop = true;
    unlock();
    vehicle_notify_feedback();
}
void vehicle_invalidate_line(void)
{
    if (!lock()) return;
    data.line_valid = FALSE;
    unlock();
    vehicle_notify_feedback();
}
BOOL vehicle_request_reset(void)
{
    if (!lock()) return FALSE;
    S2_STATUS motor = {{0}};
    bool busy = false;
    BOOL ok = !vc_core_running(&model) && !pending.start && !pending.stop && !pending.reset &&
              motion_get_feedback(&motor, NULL, &busy);
    if (ok) {
        pending.reset = true;
        reset_started = now_ms();
        reset_refreshed = reset_started - VC_REFRESH_MS;
        pending.auto_start = false;
        motion_stop();
    }
    unlock();
    if (ok) vehicle_notify_feedback();
    return ok;
}
void vehicle_request_stop(void)
{
    if (!lock()) return;
    pending.stop = true;
    pending.start = pending.reset = pending.auto_start = false;
    motion_stop();
    unlock();
    vehicle_notify_feedback();
}

static BOOL link_ready(void)
{
#if TM_MQTT
    return (s1_connection_status() & 7u) == 7u;
#else
    return TRUE;
#endif
}

static VC_INPUT controller_input(BOOL ready, BOOL linked, unsigned faults, bool busy, uint32_t now)
{
    return (VC_INPUT){
        .start = pending.start, .stop = pending.stop, .reset = pending.reset,
        .ready = ready, .link_ok = linked, .motor_faults = faults, .motor_busy = busy,
        .line_stop = !vc_core_detouring(&model) && (data.line_stop || pending.line_stop),
        .line_valid = data.line_valid,
        .line_age_ms = now - data.line_timestamp_ms,
        .left_rpm = data.left_rpm, .right_rpm = data.right_rpm,
        .obstacle_blocked = obstacle_blocked(),
        .avoidance = {
            .now = now, .range_at = obstacle_range_at, .range_mm = obstacle_range.range_mm,
            .range_valid = range_valid(now), .profile_at = full_profile_at,
            .profile = full_profile, .profile_valid = pending.profile_valid,
            .retry_scan = pending.start,
            .line_at = data.line_timestamp_ms, .line_hit = line_hit(),
            .left_on_tape = data.l >= S3_EDGE_DARKNESS,
            .right_on_tape = data.r >= S3_EDGE_DARKNESS,
            .line_center_seen = data.c >= S5_CENTER_TAPE_DARK,
            .line_junction = data.l >= S5_WIDE_MARKING && data.r >= S5_WIDE_MARKING,
        },
    };
}
/* Use the existing startup blocker on the wire; no telemetry format change. */
static VC_START_BLOCK start_block(const VC_INPUT *input)
{
    VC_START_BLOCK block = vc_core_start_block(&model, input);
    if (block == VC_START_READY && input->drive_failed) return VC_START_MOTOR;
    if (block == VC_START_READY && !b4_safety_restart(0U)) return VC_START_STARTUP;
    return block;
}
BOOL vehicle_request_start(void)
{
    if (!lock()) return FALSE;
    S2_STATUS motor = {{0}};
    bool busy = false;
    BOOL ready = motion_get_feedback(&motor, NULL, &busy);
    VC_INPUT input = controller_input(ready, link_ready(), s2_get_bits(&motor, 5, 4), busy, now_ms());
    VC_START_BLOCK block = start_block(&input);
    data.last_start_reject = block;
    BOOL ok = block == VC_START_READY;
    if (ok && (!vc_core_running(&model) || model.avoidance.phase == S5_WAIT_PATH)) {
        pending.start = true;
        pending.auto_start = false;
    }
    unlock();
    if (ok) vehicle_notify_feedback();
    else tm_printf((UB*)"[VEHICLE] START rejected: blocker=%u\n", (unsigned)block);
    return ok;
}
BOOL vehicle_submit_terrain(unsigned height,unsigned phase)
{
    if(height>2000||phase>5||!lock())return FALSE;
    data.height_mm=height;data.terrain_phase=phase;
    data.terrain_timestamp_ms=now_ms();data.terrain_valid=TRUE;unlock();return TRUE;
}
void vehicle_enable_obstacle_monitor(void)
{
    if (!lock()) return;
    pending.obstacle_monitor = true;
    pending.last_echo = S5_ECHO_TIMING;
    pending.echo_grace_used = false;
    pending.follow_blocked = false;
    obstacle_range = (S5_RANGE_GATE){.range_mm = S5_UNKNOWN_RANGE, .blocked = true};
    unlock();
    vehicle_notify_feedback();
}

static void submit_obstacle_reading(unsigned range, S5_ECHO_STATE echo)
{
    if (!lock()) return;
    uint32_t now = now_ms();
    if (s5_avoid_executing(model.avoidance.phase)) {
        unlock();
        return;
    }
    bool hold = echo == S5_ECHO_QUIET && !pending.echo_grace_used &&
                s5_range_available(obstacle_range.range_mm) && !obstacle_range.blocked &&
                now - obstacle_range_at <= VC_ECHO_GRACE_MS;
    pending.last_echo = echo;
    bool accepted = false;
    if (hold) {
        pending.echo_grace_used = true;
        obstacle_range.first_open = false;
    } else {
        accepted = s5_accept_range(&obstacle_range, range);
    }
    if (accepted) {
        obstacle_range_at = now;
        pending.echo_grace_used = false;
        /* Only a measured range can trigger a close obstacle. An unknown
           range cannot clear a close obstacle that was already detected. */
        if (s5_range_available(range)) {
            unsigned follow_limit = pending.follow_blocked ? S5_FOLLOW_CLEAR_MM : S5_FOLLOW_STOP_MM;
            pending.follow_blocked = range <= follow_limit;
        }
        if (!vc_core_detouring(&model) && !data.obstacle_profile) {
            data.obstacle_valid = s5_range_measured(range);
            if (data.obstacle_valid) {
                data.near_mm = range;
                data.width_mm = S5_UNKNOWN_WIDTH;
                data.clear_left_mm = data.clear_right_mm = S5_UNKNOWN_RANGE;
                data.obstacle_angle_offset = S5_CENTRE_ANGLE - S5_MIN_ANGLE;
                data.obstacle_timestamp_ms = obstacle_range_at;
            }
        }
    }
    unlock();
    if (accepted) vehicle_notify_feedback();
}

void vehicle_submit_obstacle_range(unsigned range)
{
    submit_obstacle_reading(range, S5_ECHO_PULSE);
}

void vehicle_submit_obstacle_ping(S5_PING ping)
{
    submit_obstacle_reading(ping.range_mm, ping.state);
}

BOOL vehicle_obstacle_ranging_paused(void)
{
    if (!lock()) return FALSE;
    BOOL paused = s5_avoid_executing(model.avoidance.phase);
    unlock();
    return paused;
}

static BOOL scan_wanted(void)
{
    return model.avoidance.phase == S5_PROFILE_PATH && !pending.profile_valid &&
           !pending.stop && !pending.line_stop;
}

BOOL vehicle_obstacle_scan_request(uint32_t *request, unsigned *limit_mm)
{
    if (!request || !lock()) return FALSE;
    BOOL allowed = scan_wanted() && pending.scan == SCAN_READY;
    if (allowed) {
        *request = scan_request_id;
        if (limit_mm) *limit_mm = s5_scan_range_limit(&model.avoidance);
        pending.scan = SCAN_ACTIVE;
    }
    unlock();
    return allowed;
}

BOOL vehicle_obstacle_scan_current(uint32_t request)
{
    if (!lock()) return FALSE;
    BOOL current = scan_wanted() && pending.scan == SCAN_ACTIVE && scan_request_id == request;
    unlock();
    return current;
}

void vehicle_finish_obstacle_scan(const S5_PROFILE *profile, uint32_t request)
{
    if (!lock()) return;
    BOOL current = scan_wanted() && model.avoidance.phase == S5_PROFILE_PATH &&
                   pending.scan == SCAN_ACTIVE && scan_request_id == request;
    if (current) {
        uint32_t now = now_ms();
        pending.scan = SCAN_OFF;
        if (profile && s5_range_measured(profile->nearest_mm)) {
            data.obstacle_valid = data.obstacle_profile = true;
            data.near_mm = profile->nearest_mm;
            data.width_mm = profile->width_mm;
            data.clear_left_mm = profile->left_path_mm;
            data.clear_right_mm = profile->right_path_mm;
            data.obstacle_angle_offset = profile->angle_offset_deg;
            data.obstacle_timestamp_ms = now;
        }
        full_profile = profile ? *profile : (S5_PROFILE){.nearest_mm = S5_UNKNOWN_RANGE};
        full_profile_at = now;
        pending.profile_valid = true;
    }
    unlock();
    if (current) vehicle_notify_feedback();
}
BOOL vehicle_get_snapshot(VEHICLE_SNAPSHOT *out)
{
    if(!out)return FALSE;
    if(!lock())return FALSE;
    S2_STATUS motor={{0}};
    bool busy=false;
    BOOL ready=motion_get_feedback(&motor,NULL,&busy),linked=link_ready();
    *out=data;out->timestamp_ms=now_ms();out->motor_ready=ready;out->link_ready=linked;
    out->start_pending=pending.start;out->stop_pending=pending.stop;out->reset_pending=pending.reset;
    S5_PHASE phase = model.avoidance.phase;
    bool executing = s5_avoid_executing(phase);
    out->obstacle_blocked = !executing && obstacle_blocked();
    /* Forward samples naturally age out during the initial sweep. */
    bool scanning = phase == S5_PROFILE_PATH && !pending.profile_valid;
    bool echo_fault = phase == S5_FAILED && model.avoidance.failure == S5_FAIL_ECHO;
    out->obstacle_no_echo = pending.obstacle_monitor &&
                            (echo_fault || (!executing && !scanning && !range_valid(out->timestamp_ms)));
    out->avoidance_phase = model.avoidance.phase;
    out->avoidance_side = model.avoidance.side;
    out->avoidance_failure = model.avoidance.failure;
    out->avoidance_failed_phase = model.avoidance.failed_phase;
    out->obstacle_forward_mm = range_valid(out->timestamp_ms) ? obstacle_range.range_mm : S5_UNKNOWN_RANGE;
    out->motor_faults=s2_get_bits(&motor,5,4);
    out->timing_fault_gap_us=s2_core_timing_fault_gap_us();
    out->control_gap_limit_us=MAX_CONTROL_GAP_US;
    VC_INPUT input = controller_input(ready, linked, out->motor_faults, busy, out->timestamp_ms);
    out->start_block = start_block(&input);
    unlock();return TRUE;
}
BOOL vehicle_is_following(void)
{
    if (!lock()) return FALSE;
    BOOL following = model.state == VC_FOLLOW_LINE;
    unlock();
    return following;
}
void vehicle_record_telemetry_drop(void)
{ if(lock()){if(data.telemetry_drops!=0xffffffffu)data.telemetry_drops++;unlock();} }
static bool tracking_straight;
static uint32_t straight_since;
static S5_MOTION drive_feedback(const S2_STATUS *status, const S2_REPLY *reply,
                                bool busy, uint32_t now)
{
    if (!drive.command) return S5_MOTION_NONE;
    bool unanswered = reply->sequence == drive.sequence;
    if (unanswered && now - drive.issued_at < VC_ACTION_TIMEOUT_MS) return S5_MOTION_QUEUED;
    bool matched = !unanswered && reply->command == drive.command;
    if (drive.velocity) {
        drive.command = 0;
        drive.retry = matched && !reply->accepted && reply->reason == S2_REPLY_SETTLING;
        drive.failed = !matched || (!reply->accepted && !drive.retry);
        return S5_MOTION_NONE;
    }
    if (!matched || !reply->accepted) return S5_MOTION_FAILED;
    if (busy) return S5_MOTION_PENDING;
    if ((drive.command & 15u) == S2_ARC)
        return s2_core_arc_finished((drive.command >> 4) != 0, status) ? S5_MOTION_DONE : S5_MOTION_FAILED;
    return s2_core_maneuver_finished(drive.command >> 4, status) ? S5_MOTION_DONE : S5_MOTION_FAILED;
}

static void submit_output(const VC_INPUT *in, const VC_OUTPUT *out, const S2_REPLY *reply,
                          bool detour)
{
    if (out->stop || (!detour && !out->left_rpm && !out->right_rpm)) {
        if (!in->stop && (drive.moving || out->maneuver.action == S5_STOP)) motion_stop();
        drive = (__typeof__(drive)){0};
        return;
    }

    bool submitted;
    if (detour) {
        if (in->avoidance.motion == S5_MOTION_DONE) {
            drive.command = 0;
            drive.moving = false;
        }
        S5_ACTION action = out->maneuver.action;
        if (action == S5_WAIT || action == S5_RESCAN) return;
        if (action == S5_REVERSE)
            submitted = motion_maneuver_reverse(out->maneuver.distance_mm, &drive.command);
        else if (action == S5_ADVANCE)
            submitted = motion_maneuver(out->maneuver.distance_mm, false, &drive.command);
        else
            submitted = motion_arc(action == S5_ARC_RIGHT, &drive.command);
        drive.velocity = false;
    } else {
        bool refresh = !drive.moving || in->avoidance.now - drive.refreshed >= VC_REFRESH_MS;
        bool changed = out->left_rpm != drive.left_rpm || out->right_rpm != drive.right_rpm;
        if (drive.command || !(refresh || (changed && !drive.retry))) return;
        submitted = motion_set_wheel_rpm_tracked(out->left_rpm, out->right_rpm, &drive.command);
        drive.velocity = true;
        drive.left_rpm = out->left_rpm;
        drive.right_rpm = out->right_rpm;
        drive.refreshed = in->avoidance.now;
    }
    drive.failed = !submitted;
    if (submitted) {
        drive.moving = true;
        drive.sequence = reply->sequence;
        drive.issued_at = in->avoidance.now;
    }
}

static const char *const avoidance_phase_name[] = {
    [S5_OFF] = "idle",
    [S5_PROFILE_PATH] = "scanning for an obstacle or a clear path",
    [S5_TURN_OUT] = "turning off the line",
    [S5_SHIFT_OUT] = "leaving the line for the bypass",
    [S5_ALIGN_OUTSIDE] = "aligning for bypass",
    [S5_PASS] = "passing obstacle",
    [S5_TURN_IN] = "turning toward original line",
    [S5_FIND_LINE] = "searching for the line",
    [S5_CENTER_LINE] = "positioning for the final arc",
    [S5_ALIGN_LINE] = "realigning with the line",
    [S5_COMPLETE] = "line reacquired, resuming line following",
    [S5_FAILED] = "detour stopped",
    [S5_CHECK_LINE] = "confirming the line",
    [S5_BACK_UP] = "reversing for another scan",
    [S5_WAIT_PATH] = "waiting to verify a route",
};

static TMO vehicle_service_once(void)
{
    if (!lock()) return VC_REFRESH_MS;
    S2_STATUS status = {{0}};
    S2_REPLY reply = {0};
    bool busy = false;
    BOOL ready = motion_get_feedback(&status, &reply, &busy);
    uint32_t now = now_ms();
    VC_INPUT in = controller_input(ready, link_ready(), s2_get_bits(&status, 5, 4), busy, now);
    VC_OUTPUT out;
    in.avoidance.motion = drive_feedback(&status, &reply, busy, now);
    in.drive_failed = drive.failed;
    unsigned left_measured = s2_get_bits(&status, 9, 9);
    unsigned right_measured = s2_get_bits(&status, 18, 9);
    in.avoidance.stopped = ready && !busy && left_measured <= 2 && right_measured <= 2;
    bool line_fresh = in.line_valid && in.line_age_ms <= VC_LINE_MAX_AGE_MS;
    /* The outer sensors straddle the tape on a straight. This reading alone
       cannot confirm tape after a detour. */
    in.avoidance.line_centered = line_fresh && !in.avoidance.line_hit &&
                                data.l < S5_LINE_DARK && data.r < S5_LINE_DARK;
    if (model.avoidance.phase == S5_FIND_LINE) {
        unsigned min_cross_mm = model.avoidance.search_after_mm;
        bool distance_command = in.avoidance.motion == S5_MOTION_PENDING || in.avoidance.motion == S5_MOTION_DONE;
        in.avoidance.near_original_line = distance_command &&
              (!min_cross_mm || s2_core_distance_reached(min_cross_mm, &status));
        if (distance_command) {
            unsigned progress = s2_core_distance_travelled_mm(&status);
            in.avoidance.search_progress_mm = progress <= S5_MAX_LEG_MM ? progress : S5_MAX_LEG_MM;
        }
        if (line_fresh && distance_command) {
            if (data.l > tape_search_peak.left) tape_search_peak.left = data.l;
            if (data.c > tape_search_peak.center) tape_search_peak.center = data.c;
            if (data.r > tape_search_peak.right) tape_search_peak.right = data.r;
        }
    } else if (model.avoidance.phase == S5_CENTER_LINE && in.avoidance.motion == S5_MOTION_NONE) {
        /* STOP preserves the search command's encoder counts until another move starts. */
        unsigned progress = s2_core_distance_travelled_mm(&status);
        in.avoidance.search_progress_mm = progress <= S5_MAX_LEG_MM ? progress : S5_MAX_LEG_MM;
    }
    bool straight = model.state == VC_FOLLOW_LINE && in.avoidance.line_centered &&
                    in.left_rpm == in.right_rpm &&
                    left_measured >= 5u && right_measured >= 5u &&
                    left_measured <= right_measured + 3 && right_measured <= left_measured + 3;
    if (straight && !tracking_straight) straight_since = now;
    tracking_straight = straight;
    in.avoidance.can_retrace = straight && now - straight_since >= VC_RETRACE_MIN_MS;
    in.avoidance.can_retrace_far = straight && now - straight_since >= VC_RETRACE_LONG_MS;
    VC_START_BLOCK blocker = start_block(&in);
    if (pending.auto_start && blocker == VC_START_READY) {
        in.start = TRUE;
        pending.auto_start = false;
    }
    /* Only an explicit START accepted by the worker rearms Buddy 4. The
     * sampler releases its mutex before requesting a controller stop. */
    if (pending.start && blocker == VC_START_READY &&
        !b4_safety_restart(1U)) blocker = VC_START_STARTUP;
    if (blocker != VC_START_READY) in.start = false;
    if (pending.start && blocker != VC_START_READY) data.last_start_reject = blocker;
    pending.start = pending.stop = pending.line_stop = false;
    unsigned previous_reason = model.reason;
    S5_PHASE previous = model.avoidance.phase;
    bool was_executing = s5_avoid_executing(previous);
    vc_core_step(&model, &in, &out);
    bool reset_done = false;
    if (in.reset) {
        if (in.avoidance.stopped && !in.motor_faults) {
            reset_done = true;
            /* Clear the fault, but require tape evidence before a new START after a failed detour. */
            bool reposition = model.reposition_required || model.avoidance.phase == S5_FAILED;
            model = (VC_MODEL){.reposition_required = reposition};
            pending.profile_valid = false;
            pending.scan = SCAN_OFF;
            tracking_straight = false;
            data.obstacle_profile = data.obstacle_valid = false;
            data.last_start_reject = VC_START_READY;
        } else if (now - reset_started >= VC_RESET_TIMEOUT_MS) {
            reset_done = true;
            motion_stop();
        } else if (in.avoidance.stopped && now - reset_refreshed >= VC_REFRESH_MS) {
            (void)motion_clear_faults();
            reset_refreshed = now;
        }
        if (reset_done) pending.reset = false;
    }
    S5_PHASE phase = model.avoidance.phase;
    unsigned side = model.avoidance.side;
    unsigned offset = model.avoidance.offset_mm;
    unsigned pass = model.avoidance.pass_mm;
    unsigned left_entry = model.avoidance.left_edge.entry_mm;
    unsigned right_entry = model.avoidance.right_edge.entry_mm;
    unsigned align_at = model.avoidance.align_at_mm;
    unsigned search_progress = model.avoidance.search_progress_mm;
    unsigned search_limit = model.avoidance.search_mm;
    unsigned left_samples = model.avoidance.left_edge.samples;
    unsigned right_samples = model.avoidance.right_edge.samples;
    if (phase == S5_FIND_LINE && phase != previous)
        tape_search_peak = (__typeof__(tape_search_peak)){0};
    bool executing = s5_avoid_executing(phase);
    if (executing && !was_executing) {
        /* The next forward range must be from the new heading after rejoining. */
        obstacle_range = (S5_RANGE_GATE){.range_mm = S5_UNKNOWN_RANGE, .blocked = true};
        pending.follow_blocked = pending.echo_grace_used = false;
    }
    /* Serialize dispatch with the request latch, so STOP can cancel an entire
       batch before any later movement is submitted. */
    submit_output(&in, &out, &reply, vc_core_detouring(&model));
    if (phase != previous || out.maneuver.action == S5_RESCAN) {
        pending.scan = SCAN_OFF;
        if (phase == S5_PROFILE_PATH) pending.profile_valid = false;
    }
    if (phase == S5_PROFILE_PATH && previous != S5_PROFILE_PATH && previous != S5_BACK_UP &&
        previous != S5_WAIT_PATH && data.obstacle_profile) {
        data.obstacle_profile = data.obstacle_valid = false;
    }
    bool scan = scan_wanted() && pending.scan == SCAN_OFF && in.avoidance.stopped && !drive.moving &&
                out.maneuver.action != S5_STOP;
    if (scan) {
        ++scan_request_id;
        pending.scan = SCAN_READY;
    }
    data.state = model.state;
    unsigned reason = data.reason = model.reason;
    unsigned failure = model.avoidance.failure;
    unsigned last_echo = pending.last_echo;
    data.motor_faults = in.motor_faults;
    TMO delay = pending.reset ? (TMO)VC_REFRESH_MS : TMO_FEVR;
    if (vc_core_running(&model)) {
        delay = VC_REFRESH_MS;
        if (vc_core_needs_line(&model) && line_fresh) {
            uint32_t line_remaining = VC_LINE_MAX_AGE_MS + 1u - in.line_age_ms;
            if (line_remaining < (uint32_t)delay) delay = line_remaining;
        }
        if (in.avoidance.range_valid) {
            uint32_t range_remaining = S5_RANGE_MAX_AGE_MS + 1u - (now - in.avoidance.range_at);
            if (range_remaining < (uint32_t)delay) delay = range_remaining;
        }
    }
    bool dispatch_failed = drive.failed;
    unlock();

    if (scan || (was_executing && !executing)) subsystem5_wake();
    if (dispatch_failed) vehicle_notify_feedback();
    if (reason == VC_REASON_STALE_LINE && previous_reason != VC_REASON_STALE_LINE) {
        tm_printf((UB*)"[VC] stopped: local IR sample %s, last update %u ms ago\n",
                  in.line_valid ? "expired" : "invalid", (unsigned)in.line_age_ms);
        s3_log_sampler_state();
    }
    if (out.maneuver.action == S5_RESCAN)
        tm_printf((UB*)"[S5] scan inconclusive, retrying while stopped\n");
    if (!dispatch_failed && (out.maneuver.action == S5_ARC_LEFT || out.maneuver.action == S5_ARC_RIGHT))
        tm_printf((UB*)"[S5] commanding %u deg rolling turn %s\n", S2_ARC_ANGLE_DEG,
                  out.maneuver.action == S5_ARC_LEFT ? "left" : "right");
    if (previous != phase) {
        tm_printf((UB*)"[S5] avoidance phase=%u side=%u at %u ms: %s\n",
                  (unsigned)phase, side, (unsigned)now, avoidance_phase_name[phase]);
        if (phase == S5_TURN_OUT)
            tm_printf((UB*)"[S5] planned bypass %s, sideways target=%u mm, straight pass=%u mm\n",
                      side == S5_LEFT ? "T (left)" : "R (right)", offset, pass);
        if (phase == S5_CENTER_LINE)
            tm_printf((UB*)"[S5] tape crossings left=%u mm right=%u mm, final arc starts at %u mm\n",
                      left_entry, right_entry, align_at);
        if (phase == S5_FAILED || phase == S5_WAIT_PATH)
            tm_printf((UB*)"[S5] stopped during %s, detour reason=%u vehicle reason=%u, "
                          "range=%u mm, echo state=%u, motor faults=%u, encoder L=%u R=%u\n",
                      avoidance_phase_name[previous], failure, reason,
                      in.avoidance.range_valid ? (unsigned)in.avoidance.range_mm : S5_UNKNOWN_RANGE,
                      last_echo, in.motor_faults, s2_get_bits(&status,41,13), s2_get_bits(&status,54,13));
        if (previous == S5_FIND_LINE && (failure == S5_FAIL_LINE || reason == VC_REASON_STALE_LINE))
            tm_printf((UB*)"[S5] tape search %u/%u mm, edge samples left=%u right=%u, IR peak L/C/R=%u/%u/%u\n",
                      search_progress, search_limit, left_samples, right_samples,
                      tape_search_peak.left, tape_search_peak.center, tape_search_peak.right);
    }
    return delay;
}
static void vehicle_task(INT stacd,void *exinf)
{
    (void)stacd;(void)exinf;
    motion_stop();
    for (;;) {
        TMO delay = vehicle_service_once();
        UINT events;
        (void)tk_wai_flg(work_event, VC_WAKE, TWF_ORW | TWF_BITCLR, &events, delay);
    }
}
ER vehicle_start_task(void)
{ ID id=app_start_task((FP)vehicle_task,15,TP_PRC1,"vehicle_controller");return id<E_OK?id:E_OK; }
