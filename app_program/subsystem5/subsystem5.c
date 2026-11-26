#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"

#ifdef CPU_RP2040
#define SERVO_SETTLE_MS 120
#define SERVO_SWEEP_SETTLE_MS 240
#define SERVO_HOME_SETTLE_MS 350
#define PING_INTERVAL_MS 60
#define FINE_ATTEMPTS 5u
#define CLEARANCE_RETRY_PAUSE_MS 200
#define RANGE_LOG_INTERVAL_MS 2000u
#define SCAN_WAKE 1u
#define BYPASS_SCAN_STEPS (S5_CLEARANCE_BEARING_DEG / S5_FINE_STEP)
#define OUTWARD_SCAN_STEPS (S5_OUTWARD_CLEARANCE_BEARING_DEG / S5_FINE_STEP)

_Static_assert(S5_CLEARANCE_BEARING_DEG % S5_FINE_STEP == 0 && BYPASS_SCAN_STEPS <= S5_CENTRE_INDEX,
               "The sweep must cover both clearance bearings");
_Static_assert(S5_OUTWARD_CLEARANCE_BEARING_DEG % S5_FINE_STEP == 0 &&
               OUTWARD_SCAN_STEPS <= S5_CENTRE_INDEX && S5_CENTRE_INDEX + OUTWARD_SCAN_STEPS < S5_SCAN_POINTS,
               "The sweep must cover both outward clearance bearings");

static ID scan_event;
static unsigned servo_angle = S5_CENTRE_ANGLE;
typedef enum { SCAN_COARSE, SCAN_FINE, SCAN_CLEARANCE } SCAN_READING;
static const char *const echo_status[] = {
    [S5_ECHO_QUIET] = "rise timeout",
    [S5_ECHO_PULSE] = "pulse",
    [S5_ECHO_HIGH] = "high before trigger",
    [S5_ECHO_TIMING] = "timing fault",
    [S5_ECHO_FALL_TIMEOUT] = "fall timeout",
    [S5_ECHO_LATE] = "late fall",
    [S5_ECHO_NO_TARGET] = "no target",
};
static const char *const width_status[] = {
    [S5_WIDTH_OK] = "no width estimate",
    [S5_WIDTH_LEFT_EDGE] = "no verified background near left edge",
    [S5_WIDTH_RIGHT_EDGE] = "no verified background near right edge",
    [S5_WIDTH_BOTH_EDGES] = "no verified background near either edge",
    [S5_WIDTH_UNRESOLVED] = "edge echoes do not give a width within 1 to 2000 mm",
};

static void aim(unsigned angle)
{
    unsigned change = angle > servo_angle ? angle - servo_angle : servo_angle - angle;
    s5_hw_angle(angle);
    servo_angle = angle;
    tk_dly_tsk(change > 15u ? SERVO_SWEEP_SETTLE_MS : SERVO_SETTLE_MS);
}

static void log_ping(const char *tag, const char *context, unsigned angle, S5_PING ping)
{
    if (s5_range_measured(ping.range_mm))
        tm_printf((UB*)"%s %s angle=%u echo=%u us range=%u mm\n",
                  tag, context, angle, (unsigned)ping.pulse_us, (unsigned)ping.range_mm);
    else
        tm_printf((UB*)"%s %s angle=%u echo=%s pulse=%u us range=%s\n",
                  tag, context, angle, echo_status[ping.state], (unsigned)ping.pulse_us,
                  ping.range_mm == S5_OPEN_RANGE ? "open" : "unavailable");
}

static unsigned read_at(unsigned index, BOOL verbose, SCAN_READING reading, unsigned edge_reference)
{
    aim(s5_scan_bearing(index));
    unsigned previous = S5_UNKNOWN_RANGE;
    const unsigned attempts = reading == SCAN_COARSE ? 3u : FINE_ATTEMPTS;
    bool edge = s5_range_measured(edge_reference);
    unsigned ranges[FINE_ATTEMPTS];
    S5_PING clearance_pings[FINE_ATTEMPTS];
    for (unsigned attempt = 0; attempt < attempts; ++attempt) {
        /* Apply the longer cadence and capture window only to the final
           two pings at an unresolved side bearing while stopped. */
        bool extended = reading == SCAN_CLEARANCE && attempt >= FINE_ATTEMPTS - 2u;
        if (attempt) tk_dly_tsk(extended ? CLEARANCE_RETRY_PAUSE_MS : PING_INTERVAL_MS);
        S5_PING ping = extended ? s5_hw_ping_extended() : s5_hw_ping();
        if (verbose) log_ping("[S5 BENCH]", attempt ? "recheck" : "scan", servo_angle, ping);
        if (reading == SCAN_COARSE && s5_range_measured(ping.range_mm)) return ping.range_mm;
        if (reading == SCAN_CLEARANCE) clearance_pings[attempt] = ping;

        unsigned candidate = ping.range_mm;
        ranges[attempt] = candidate;
        unsigned confirmed = s5_confirm_scan_range(previous, candidate);
        if (s5_range_available(confirmed) && (!edge || !s5_background_beyond(confirmed, edge_reference))) {
            return reading == SCAN_CLEARANCE ? s5_clearance_range(ranges, attempt + 1u) : confirmed;
        }
        /* A missed ping can separate two matching measured echoes. */
        if (!s5_range_available(candidate) && s5_range_measured(previous)) continue;
        previous = candidate;
    }
    if (reading == SCAN_CLEARANCE) {
        unsigned range = s5_clearance_range(ranges, attempts);
        if (!verbose && range == S5_UNKNOWN_RANGE) {
            for (unsigned i = 0; i < attempts; ++i)
                log_ping("[S5]", "clearance attempt", servo_angle, clearance_pings[i]);
        }
        return range;
    }
    return edge ? s5_edge_background(ranges, attempts, edge_reference) : S5_UNKNOWN_RANGE;
}

/* Consecutive missing bearings bound the scan work in the driving profile. */
static unsigned scan_edge(S5_SCAN *samples, unsigned anchor, unsigned reference,
                          int step, BOOL verbose, const uint32_t *request)
{
    int index = (int)anchor;
    unsigned misses = 0;
    while (index + step >= 0 && index + step < (int)S5_SCAN_POINTS &&
           (verbose || misses < 4u)) {
        if (request && !vehicle_obstacle_scan_current(*request)) break;
        index += step;
        unsigned range = read_at((unsigned)index, verbose, SCAN_FINE, reference);
        s5_scan_set(samples, (unsigned)index, range);
        const char *side = step < 0 ? "left" : "right";
        if (s5_background_beyond(range, reference)) {
            if (verbose)
                tm_printf((UB*)"[S5 BENCH] %s edge confirmed at angle=%u deg\n", side, servo_angle);
            return (unsigned)index;
        } else if (s5_same_obstacle(range, reference)) {
            misses = 0;
            if (verbose)
                tm_printf((UB*)"[S5 BENCH] %s angle=%u deg: %u mm grouped with %u mm target\n",
                          side, servo_angle, range, reference);
        } else {
            ++misses;
            if (verbose)
                tm_printf((UB*)"[S5 BENCH] %s angle=%u deg: edge unconfirmed\n", side, servo_angle);
        }
    }
    if (verbose)
        tm_printf((UB*)"[S5 BENCH] %s sweep limit at %u deg: no confirmed edge\n",
                  step < 0 ? "left" : "right", servo_angle);
    return (unsigned)index;
}

/* The last sweep bearing can be far from centre. Give the forward reading
   several chances after settling, and feed each ping through the range gate. */
static void refresh_forward(uint32_t request)
{
    bool open_last = false;
    for (unsigned attempt = 0; attempt < 3u; ++attempt) {
        if (!vehicle_obstacle_scan_current(request)) return;
        S5_PING ping = s5_hw_ping();
        unsigned range = ping.range_mm;
        vehicle_submit_obstacle_ping(ping);
        if (s5_range_measured(range) || (open_last && range == S5_OPEN_RANGE)) return;
        open_last = range == S5_OPEN_RANGE;
        if (attempt + 1u < 3u) tk_dly_tsk(PING_INTERVAL_MS);
    }
}

static BOOL scan_obstacle(S5_PROFILE *profile, BOOL verbose, const uint32_t *request, unsigned limit_mm)
{
    const char *tag = verbose ? "[S5 BENCH]" : "[S5]";
    S5_SCAN samples = {0};
    tm_printf((UB*)"%s confirming forward target before GP12 coarse sweep\n", tag);
    unsigned centre = read_at(S5_CENTRE_INDEX, verbose, SCAN_FINE, S5_UNKNOWN_RANGE);
    s5_scan_set(&samples, S5_CENTRE_INDEX, centre);
    if (request && !vehicle_obstacle_scan_current(*request)) return FALSE;
    if (request && !s5_range_available(centre)) {
        vehicle_submit_obstacle_range(S5_UNKNOWN_RANGE);
        return FALSE;
    }
    if (request && s5_range_available(centre) && centre > limit_mm) {
        vehicle_submit_obstacle_range(centre);
        if(centre==S5_OPEN_RANGE)vehicle_submit_obstacle_range(centre); /* read_at confirmed it */
        *profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        return TRUE;
    }
    for (unsigned i = S5_COARSE_FIRST; i <= S5_COARSE_LAST; i += S5_COARSE_STEP) {
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
        if (i == S5_CENTRE_INDEX && s5_range_available(centre)) continue;
        s5_scan_set(&samples, i, read_at(i, verbose, SCAN_COARSE, S5_UNKNOWN_RANGE));
    }

    unsigned anchor = s5_scan_target(&samples, limit_mm);
    if (anchor == S5_NO_TARGET) {
        aim(S5_CENTRE_ANGLE);
        unsigned forward = read_at(S5_CENTRE_INDEX, FALSE, SCAN_FINE, S5_UNKNOWN_RANGE);
        if (!s5_range_available(forward) || forward <= limit_mm) return FALSE;
        if(request) {
            vehicle_submit_obstacle_range(forward);
            if(forward==S5_OPEN_RANGE)vehicle_submit_obstacle_range(forward);
        }
        *profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        tm_printf((UB*)"%s no obstacle in the coarse sweep\n", tag);
        return TRUE;
    }

    unsigned coarse = s5_scan_get(&samples, anchor);
    if (verbose)
        tm_printf((UB*)"%s fine scan starting at %u deg\n", tag, s5_scan_bearing(anchor));
    unsigned fine = anchor == S5_CENTRE_INDEX && s5_range_available(centre) ?
                    centre : read_at(anchor, verbose, SCAN_FINE, S5_UNKNOWN_RANGE);
    if (!s5_same_obstacle(fine, coarse) || fine > limit_mm) {
        tm_printf((UB*)"%s fine scan could not confirm the %u mm target at %u deg\n",
                  tag, coarse, servo_angle);
        goto cancelled;
    }
    s5_scan_set(&samples, anchor, fine);

    unsigned first = scan_edge(&samples, anchor, fine, -1, verbose, request);
    unsigned last = scan_edge(&samples, anchor, fine, 1, verbose, request);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;

    *profile = s5_profile(&samples, first, last, anchor, NULL);
    profile->left_path_mm = read_at(S5_CENTRE_INDEX - BYPASS_SCAN_STEPS, verbose,
                                    SCAN_CLEARANCE, S5_UNKNOWN_RANGE);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    profile->right_path_mm = read_at(S5_CENTRE_INDEX + BYPASS_SCAN_STEPS, verbose,
                                     SCAN_CLEARANCE, S5_UNKNOWN_RANGE);
    if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    S5_BYPASS plan = s5_plan_bypass_at_range(profile, limit_mm);
    /* Confirm a fresh corridor reading. A coarse sweep can contain only one echo. */
    if (plan.probe_left) {
        profile->left_outward_mm = read_at(S5_CENTRE_INDEX - OUTWARD_SCAN_STEPS, verbose,
                                          SCAN_CLEARANCE, S5_UNKNOWN_RANGE);
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    }
    if (plan.probe_right) {
        profile->right_outward_mm = read_at(S5_CENTRE_INDEX + OUTWARD_SCAN_STEPS, verbose,
                                           SCAN_CLEARANCE, S5_UNKNOWN_RANGE);
        if (request && !vehicle_obstacle_scan_current(*request)) goto cancelled;
    }
    aim(S5_CENTRE_ANGLE);
    if (request) refresh_forward(*request);
    tm_printf((UB*)"%s paths T(left %u deg)=%u mm R(right %u deg)=%u mm (4094=open, 4095=unavailable)\n",
              tag, S5_CENTRE_ANGLE + S5_CLEARANCE_BEARING_DEG, (unsigned)profile->left_path_mm,
              S5_CENTRE_ANGLE - S5_CLEARANCE_BEARING_DEG, (unsigned)profile->right_path_mm);
    if (plan.probe_left || plan.probe_right)
        tm_printf((UB*)"%s outward clearance T(left %u deg)=%u mm R(right %u deg)=%u mm\n",
                  tag, S5_CENTRE_ANGLE + S5_OUTWARD_CLEARANCE_BEARING_DEG, (unsigned)profile->left_outward_mm,
                  S5_CENTRE_ANGLE - S5_OUTWARD_CLEARANCE_BEARING_DEG, (unsigned)profile->right_outward_mm);
    tm_printf((UB*)"%s edge offsets left=%d mm (%s), right=%d mm (%s), front clearance=%u mm\n", tag,
              (int)profile->left_edge_mm, profile->left_edge_valid ? "estimated" : "unconfirmed",
              (int)profile->right_edge_mm, profile->right_edge_valid ? "estimated" : "unconfirmed",
              (unsigned)profile->front_mm);
    if (profile->width_status != S5_WIDTH_OK) {
        if (profile->width_mm == S5_UNKNOWN_WIDTH)
            tm_printf((UB*)"%s width=unavailable (%s) nearest_distance=%u mm\n",
                      tag, width_status[profile->width_status], (unsigned)profile->nearest_mm);
        else
            tm_printf((UB*)"%s estimated_width=%u mm from observed span (%s) nearest_distance=%u mm\n",
                      tag, (unsigned)profile->width_mm, width_status[profile->width_status],
                      (unsigned)profile->nearest_mm);
        if (profile->left_clipped)
            tm_printf((UB*)"%s left sweep limit: near return %u mm at %u deg (PWM %u us)\n",
                      tag, s5_scan_get(&samples, 0), S5_SCAN_MAX_ANGLE, s5_servo_pulse_us(S5_SCAN_MAX_ANGLE));
        if (profile->right_clipped)
            tm_printf((UB*)"%s right sweep limit: near return %u mm at %u deg (PWM %u us)\n",
                      tag, s5_scan_get(&samples, S5_SCAN_POINTS - 1), S5_SCAN_MIN_ANGLE,
                      s5_servo_pulse_us(S5_SCAN_MIN_ANGLE));
        if (!verbose) {
            for (unsigned i = first; i <= last; ++i)
                tm_printf((UB*)"[S5] width echo angle=%u deg range=%u mm\n",
                          s5_scan_bearing(i), s5_scan_get(&samples, i));
        }
    } else {
        tm_printf((UB*)"%s profile nearest_distance=%u mm width=%u mm angle=%u deg\n",
                  tag, (unsigned)profile->nearest_mm, (unsigned)profile->width_mm,
                  s5_servo_angle(profile->angle_offset_deg));
    }
    return TRUE;

cancelled:
    aim(S5_CENTRE_ANGLE);
    return FALSE;
}

#if !APP_S5_BENCH
static void log_forward(S5_PING ping, const char *context)
{
    static BOOL logged;
    static uint32_t last_log_at;
    SYSTIM time;
    tk_get_tim(&time);
    if (logged && time.lo - last_log_at < RANGE_LOG_INTERVAL_MS) return;
    logged = TRUE;
    last_log_at = time.lo;
    log_ping("[S5]", context, S5_CENTRE_ANGLE, ping);
}
#endif

static BOOL setup(void)
{
    BOOL ready = s5_hw_init();
    if (!ready)
        tm_printf((UB*)"[S5] servo or ultrasonic capture failed to start\n");
    else
        tm_printf((UB*)"[S5] GP%u centred: %u us. Waiting for sensor to settle.\n",
                  S5_SERVO_PIN, S5_SERVO_CENTRE_US);
    /* The driver holds centre as soon as PWM is enabled. Let the sensor settle
       there before accepting the first range on either firmware profile. */
    tk_dly_tsk(SERVO_HOME_SETTLE_MS);
    return ready;
}

#if !APP_S5_BENCH
static void scan_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    if (!setup()) return;
    tm_printf((UB*)"[S5] GP16 trigger, GP17 echo. Entry=%u mm, clear=%u mm\n",
              S5_FOLLOW_STOP_MM, S5_FOLLOW_CLEAR_MM);

    for (;;) {
        UINT events;
        if (vehicle_obstacle_ranging_paused()) {
            (void)tk_wai_flg(scan_event, SCAN_WAKE, TWF_ORW | TWF_BITCLR, &events, TMO_FEVR);
            continue;
        }

        uint32_t request;
        unsigned limit_mm;
        if (vehicle_obstacle_scan_request(&request, &limit_mm)) {
            S5_PROFILE profile;
            BOOL scanned = scan_obstacle(&profile, FALSE, &request, limit_mm);
            vehicle_finish_obstacle_scan(scanned ? &profile : NULL, request);
            /* Keep the normal interval after the scan's final ping. */
            tk_dly_tsk(PING_INTERVAL_MS);
            continue;
        }

        S5_PING ping = s5_hw_ping();
        vehicle_submit_obstacle_ping(ping);
        log_forward(ping, "forward");
        (void)tk_wai_flg(scan_event, SCAN_WAKE, TWF_ORW | TWF_BITCLR, &events, PING_INTERVAL_MS);
    }
}

#else
#define BENCH_PING_MS 350
#define BENCH_CONFIRMATIONS 2u
#define BENCH_TIMEOUT_CONFIRMATIONS 3u

static unsigned bench_wait_for_echo(BOOL want_target)
{
    unsigned confirmed = 0;
    unsigned timeouts = 0;
    for (;;) {
        tk_dly_tsk(BENCH_PING_MS);
        S5_PING ping = s5_hw_ping();
        log_ping("[S5 BENCH]", want_target ? "waiting for target" : "waiting to rearm",
                 S5_CENTRE_ANGLE, ping);

        unsigned range = ping.range_mm;
        bool measured = s5_range_measured(range);
        bool timed_out = !s5_range_available(range) &&
                         (ping.state == S5_ECHO_QUIET || ping.state == S5_ECHO_FALL_TIMEOUT ||
                          ping.state == S5_ECHO_LATE);
        if (!timed_out) timeouts = 0;
        else if (timeouts < BENCH_TIMEOUT_CONFIRMATIONS) ++timeouts;

        bool matches = want_target ? measured && range <= S5_SCAN_STOP_MM :
                       range == S5_OPEN_RANGE || (measured && range > S5_CLEAR_MM);
        if (matches) ++confirmed;
        else if (!want_target || timeouts != 1u) confirmed = 0;
        if (confirmed == BENCH_CONFIRMATIONS) return range;
        /* In this motor-off test a missing target can restart the detector. */
        if (!want_target && timeouts == BENCH_TIMEOUT_CONFIRMATIONS) {
            tm_printf((UB*)"[S5 BENCH] Three echo timeouts. Restarting target detection.\n");
            return S5_UNKNOWN_RANGE;
        }
    }
}

static void bench_print_clearance(const char *side, unsigned range)
{
    if (s5_range_measured(range))
        tm_printf((UB*)"[S5 BENCH] %s clearance=%u mm\n", side, range);
    else
        tm_printf((UB*)"[S5 BENCH] %s clearance=%s\n", side,
                  range == S5_OPEN_RANGE ? "open" : "unavailable");
}

static void bench_print_plan(const S5_PROFILE *profile)
{
    if (profile->nearest_mm == S5_OPEN_RANGE) {
        tm_printf((UB*)"[S5 BENCH] plan=CONTINUE turn=0 deg\n");
        return;
    }

    bench_print_clearance("left 45 deg", profile->left_path_mm);
    bench_print_clearance("right 45 deg", profile->right_path_mm);
    S5_BYPASS plan = s5_plan_bypass(profile);
    if (plan.side == S5_NO_SIDE) {
        tm_printf((UB*)"[S5 BENCH] plan=STOP reason=%u\n", (unsigned)plan.failure);
        return;
    }
    tm_printf((UB*)"[S5 BENCH] preferred=%s estimated lateral clearance needed=%u mm\n",
              plan.side == S5_LEFT ? "T (left)" : "R (right)",
              (unsigned)plan.offset_mm);
}

static void bench_task(INT stacd,void *exinf)
{
    (void)stacd;
    (void)exinf;
    tm_printf((UB*)"[S5 BENCH] GP12 servo, GP16 trigger, GP17 echo. Drive outputs off.\n");
    tm_printf((UB*)"[S5 BENCH] Scan needs two forward echoes within %u mm. One timeout is allowed between them.\n",
              S5_SCAN_STOP_MM);
    if (!setup()) return;

    /* The servo has no position feedback. Make the commanded movement easy to
       verify before trusting the bearings in the scan log. */
    const unsigned check_angles[] = {
#ifdef S5_SERVO_RIGHT_OUTER_US
        S5_SCAN_MIN_ANGLE,
#endif
        S5_MIN_ANGLE, 45u, 135u, S5_MAX_ANGLE,
#ifdef S5_SERVO_LEFT_OUTER_US
        S5_SCAN_MAX_ANGLE,
#endif
        S5_CENTRE_ANGLE,
    };
    tm_printf((UB*)"[S5 BENCH] Keep the box out of the sweep for the startup background check.\n");
    for (unsigned i = 0; i < sizeof(check_angles) / sizeof(check_angles[0]); ++i) {
        tm_printf((UB*)"[S5 BENCH] Servo check: watch sensor turn to %u deg (PWM %u us).\n",
                  check_angles[i], s5_servo_pulse_us(check_angles[i]));
        aim(check_angles[i]);
        tk_dly_tsk(1000);
        log_ping("[S5 BENCH]", "background check", check_angles[i], s5_hw_ping());
    }

    for (;;) {
        unsigned detected = bench_wait_for_echo(TRUE);
        tm_printf((UB*)"[S5 BENCH] obstacle detected at %u mm. Starting scan and turn planning.\n", detected);
        S5_PROFILE profile;
        if (scan_obstacle(&profile, TRUE, NULL, S5_CLEAR_MM))
            bench_print_plan(&profile);
        else
            tm_printf((UB*)"[S5 BENCH] plan=STOP turn=0 deg. Profile unavailable.\n");
        tm_printf((UB*)"[S5 BENCH] Servo centred. Remove target or move it beyond %u mm to rearm.\n",
                  S5_CLEAR_MM);
        (void)bench_wait_for_echo(FALSE);
        tm_printf((UB*)"[S5 BENCH] Test rearmed.\n");
    }
}
#endif
#endif

EXPORT ER subsystem5_init(void)
{
#ifdef CPU_RP2040
    T_CFLG event = {0};
    scan_event = tk_cre_flg(&event);
    if (scan_event < E_OK) return scan_event;
#endif
    return E_OK;
}

void subsystem5_wake(void)
{
#ifdef CPU_RP2040
    if (scan_event > 0) (void)tk_set_flg(scan_event, SCAN_WAKE);
#endif
}

EXPORT ER subsystem5_start(void)
{
#ifdef CPU_RP2040
#if APP_S5_BENCH
    ID task=app_start_task((FP)bench_task,S5_PRI_MAIN,TP_PRC2,"sonar_bench");
    return task<E_OK?task:E_OK;
#else
    vehicle_enable_obstacle_monitor();
    if (scan_event <= 0) return E_OBJ;
    ID task = app_start_task((FP)scan_task, S5_PRI_MAIN, TP_PRC2, "obstacle_scan");
    return task < E_OK ? task : E_OK;
#endif
#else
    return E_OK;
#endif
}
