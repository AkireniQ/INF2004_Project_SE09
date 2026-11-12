#include "subsystem5.h"

#include <math.h>
#include <stdint.h>

#include <bsp/libbsp.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#define REG32(addr) (*(volatile UW *)(uintptr_t)(addr))
#define S5_PWM_SLICE(pin)          (((pin) >> 1) & 7u)
#define S5_PWM_DIV_OFFSET          0x04u
#define S5_PWM_WRAP_OFFSET         0x08u

#define US_TRIG_PULSE_US           10u
#define US_ECHO_START_TIMEOUT_US   2500u
#define US_ECHO_MAX_US             24000u
#define US_PING_INTERVAL_US        60000u
#define US_MIN_RANGE_CM            2.0f
#define US_MAX_RANGE_CM            400.0f
#define US_SOUND_CM_PER_US         0.0343f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD(degrees) \
    ((float)(degrees) * (float)M_PI / 180.0f)

typedef enum {
    US_OK = 0,
    US_ERR_NOT_INIT,
    US_ERR_INVALID_ARG,
    US_ERR_GPIO,
    US_ERR_DELAY,
    US_ERR_ECHO_STUCK,
    US_ERR_NO_ECHO,
    US_OUT_OF_RANGE
} us_status_t;

static volatile int16_t s_angle = S5_ANGLE_CENTER_DEG;
static volatile int8_t s_sweep_direction = 1;
static volatile uint32_t s_sweep_turns;
static volatile bool s_servo_initialised;
static volatile bool s_servo_scanning;
static volatile bool s_servo_fault;
static bool s_ultrasonic_initialised;
static uint64_t s_last_ping_us;
static uint32_t s_scan_count;
static s5_sample_t s_scan_samples[S5_COARSE_POINTS];

static uint64_t timer_us(void)
{
    uint32_t high_before;
    uint32_t low;
    uint32_t high_after;

    do {
        high_before = REG32(TIMER_TIMERAWH);
        low = REG32(TIMER_TIMERAWL);
        high_after = REG32(TIMER_TIMERAWH);
    } while (high_before != high_after);

    return ((uint64_t)high_before << 32) | low;
}

static us_status_t ultrasonic_init(void)
{
    s_ultrasonic_initialised = false;
    s_last_ping_us = 0u;

    REG32(RESETS_RESET + 0x3000u) = RESETS_RESET_TIMER;
    while ((REG32(RESETS_RESET_DONE) & RESETS_RESET_TIMER) == 0u) {
    }
    REG32(0x4005802cu) = 0x20cu;
    REG32(TIMER_BASE + 0x30u) &= ~1u;

    if (gpio_set_pin(S5_ULTRASONIC_TRIG_PIN, GPIO_MODE_OUT) < E_OK ||
        gpio_set_val(S5_ULTRASONIC_TRIG_PIN, 0u) < E_OK ||
        gpio_set_pin(S5_ULTRASONIC_ECHO_PIN, GPIO_MODE_IN) < E_OK) {
        return US_ERR_GPIO;
    }

    REG32(GPIO(S5_ULTRASONIC_ECHO_PIN)) |= GPIO_PDE;
    s_ultrasonic_initialised = true;
    return US_OK;
}

static us_status_t wait_ping_interval(void)
{
    uint64_t elapsed_us;
    uint32_t remaining_ms;

    if (s_last_ping_us == 0u) {
        return US_OK;
    }

    elapsed_us = timer_us() - s_last_ping_us;
    if (elapsed_us >= US_PING_INTERVAL_US) {
        return US_OK;
    }

    remaining_ms = (uint32_t)((US_PING_INTERVAL_US - elapsed_us + 999u) /
                              1000u);
    return (tk_dly_tsk(remaining_ms) < E_OK) ? US_ERR_DELAY : US_OK;
}

static us_status_t ultrasonic_read_pulse_us(uint32_t *pulse_us,
                                             int16_t *angle_deg,
                                             int8_t *direction,
                                             uint32_t *sweep_turn)
{
    uint64_t start_us;
    uint64_t rise_us;
    uint64_t fall_us;
    us_status_t status;

    if (pulse_us == NULL || angle_deg == NULL || direction == NULL ||
        sweep_turn == NULL) {
        return US_ERR_INVALID_ARG;
    }
    *pulse_us = 0u;
    *angle_deg = s_angle;
    *direction = s_sweep_direction;
    *sweep_turn = s_sweep_turns;
    if (!s_ultrasonic_initialised) {
        return US_ERR_NOT_INIT;
    }

    status = wait_ping_interval();
    if (status != US_OK) {
        return status;
    }
    if (gpio_get_val(S5_ULTRASONIC_ECHO_PIN) != 0u) {
        return US_ERR_ECHO_STUCK;
    }

    if (gpio_set_val(S5_ULTRASONIC_TRIG_PIN, 0u) < E_OK) {
        return US_ERR_GPIO;
    }
    start_us = timer_us();
    while ((timer_us() - start_us) < 2u) {
    }
    *angle_deg = s_angle;
    *direction = s_sweep_direction;
    *sweep_turn = s_sweep_turns;

    if (gpio_set_val(S5_ULTRASONIC_TRIG_PIN, 1u) < E_OK) {
        return US_ERR_GPIO;
    }
    start_us = timer_us();
    while ((timer_us() - start_us) < US_TRIG_PULSE_US) {
    }
    if (gpio_set_val(S5_ULTRASONIC_TRIG_PIN, 0u) < E_OK) {
        return US_ERR_GPIO;
    }
    s_last_ping_us = timer_us();

    start_us = s_last_ping_us;
    while (gpio_get_val(S5_ULTRASONIC_ECHO_PIN) == 0u) {
        if ((timer_us() - start_us) > US_ECHO_START_TIMEOUT_US) {
            return US_ERR_NO_ECHO;
        }
    }

    rise_us = timer_us();
    while (gpio_get_val(S5_ULTRASONIC_ECHO_PIN) != 0u) {
        if ((timer_us() - rise_us) > US_ECHO_MAX_US) {
            *pulse_us = US_ECHO_MAX_US;
            return US_OUT_OF_RANGE;
        }
    }

    fall_us = timer_us();
    *pulse_us = (uint32_t)(fall_us - rise_us);
    return US_OK;
}

static us_status_t ultrasonic_read_cm(float *distance_cm, int16_t *angle_deg,
                                      int8_t *direction, uint32_t *sweep_turn)
{
    uint32_t pulse_us;
    us_status_t status;
    float distance;

    if (distance_cm == NULL) {
        return US_ERR_INVALID_ARG;
    }
    *distance_cm = US_MAX_RANGE_CM;

    status = ultrasonic_read_pulse_us(&pulse_us, angle_deg, direction,
                                      sweep_turn);
    if (status != US_OK && status != US_OUT_OF_RANGE) {
        return status;
    }

    distance = ((float)pulse_us * US_SOUND_CM_PER_US) / 2.0f;
    if (status == US_OUT_OF_RANGE || distance > US_MAX_RANGE_CM) {
        *distance_cm = US_MAX_RANGE_CM;
        return US_OUT_OF_RANGE;
    }
    if (distance < US_MIN_RANGE_CM) {
        return US_OUT_OF_RANGE;
    }

    *distance_cm = distance;
    return US_OK;
}

static int16_t clamp_angle(int16_t angle)
{
    if (angle < S5_ANGLE_MIN_DEG) {
        return S5_ANGLE_MIN_DEG;
    }
    if (angle > S5_ANGLE_MAX_DEG) {
        return S5_ANGLE_MAX_DEG;
    }
    return angle;
}

static uint16_t angle_to_pulse_us(int16_t angle)
{
    int16_t physical_angle = angle;
    uint32_t pulse_span = S5_SERVO_MAX_PULSE_US - S5_SERVO_MIN_PULSE_US;

#if S5_SERVO_REVERSED
    physical_angle = S5_ANGLE_MAX_DEG - angle;
#endif

    return (uint16_t)(S5_SERVO_MIN_PULSE_US +
        (pulse_span * (uint32_t)physical_angle) / S5_ANGLE_MAX_DEG);
}

ER s5_servo_init(void)
{
    uint32_t slice = S5_PWM_SLICE(S5_SERVO_PIN);
    uint32_t period_us = 1000000u / S5_SERVO_PWM_HZ;
    ER status;

    status = pwm_set_enabled(S5_SERVO_PIN, FALSE);
    if (status < E_OK) {
        return status;
    }
    status = pwm_set_pin(S5_SERVO_PIN);
    if (status < E_OK) {
        return status;
    }

    REG32(PWM_BASE + slice * 0x14u + S5_PWM_DIV_OFFSET) =
        S5_SERVO_PWM_DIVIDER << 4;
    REG32(PWM_BASE + slice * 0x14u + S5_PWM_WRAP_OFFSET) = period_us - 1u;

    status = pwm_set_cc(S5_SERVO_PIN, S5_SERVO_MIN_PULSE_US);
    if (status < E_OK) {
        return status;
    }
    status = pwm_set_enabled(S5_SERVO_PIN, TRUE);
    if (status < E_OK) {
        return status;
    }

    s_angle = S5_ANGLE_CENTER_DEG;
    s_servo_initialised = true;
    return pwm_set_cc(S5_SERVO_PIN, angle_to_pulse_us(s_angle));
}

ER s5_servo_set_angle(int16_t angle_deg)
{
    if (!s_servo_initialised) {
        return E_OBJ;
    }

    s_angle = clamp_angle(angle_deg);
    return pwm_set_cc(S5_SERVO_PIN, angle_to_pulse_us(s_angle));
}

ER s5_servo_move_to(int16_t angle_deg)
{
    int16_t target = clamp_angle(angle_deg);

    if (!s_servo_initialised) {
        return E_OBJ;
    }

    while (s_angle != target) {
        int16_t difference = target - s_angle;
        int16_t step = (difference > 0) ? S5_SERVO_STEP_DEG :
                                          -S5_SERVO_STEP_DEG;
        ER status;

        if (difference < S5_SERVO_STEP_DEG &&
            difference > -S5_SERVO_STEP_DEG) {
            step = difference;
        }
        status = s5_servo_set_angle((int16_t)(s_angle + step));
        if (status < E_OK) {
            return status;
        }
        status = tk_dly_tsk(S5_SERVO_STEP_MS);
        if (status < E_OK) {
            return status;
        }
    }
    return E_OK;
}

int16_t s5_servo_get_angle(void)
{
    return s_angle;
}

static void s5_servo_step_handler(void *exinf)
{
    int16_t next_angle;

    (void)exinf;
    if (!s_servo_scanning || s_servo_fault) {
        return;
    }

    next_angle = s_angle;
    if (s_sweep_direction > 0) {
        if (next_angle >= S5_SCAN_LEFT_DEG) {
            s_sweep_direction = -1;
            s_sweep_turns++;
            next_angle--;
        } else {
            next_angle++;
        }
    } else if (next_angle <= S5_SCAN_RIGHT_DEG) {
        s_sweep_direction = 1;
        s_sweep_turns++;
        next_angle++;
    } else {
        next_angle--;
    }

    s_angle = next_angle;
    if (pwm_set_cc(S5_SERVO_PIN, angle_to_pulse_us(next_angle)) < E_OK) {
        s_servo_fault = true;
    }
}

static void clear_scan_samples(void)
{
    for (size_t i = 0; i < S5_COARSE_POINTS; i++) {
        s_scan_samples[i].angle_deg = (int16_t)(
            S5_SCAN_RIGHT_DEG + i * S5_COARSE_STEP_DEG);
        s_scan_samples[i].distance_cm = US_MAX_RANGE_CM;
        s_scan_samples[i].valid = false;
    }
}

static void store_scan_sample(int16_t angle_deg, float distance_cm,
                              bool valid)
{
    size_t index;

    if (angle_deg < S5_SCAN_RIGHT_DEG || angle_deg > S5_SCAN_LEFT_DEG) {
        return;
    }

    index = (size_t)(angle_deg - S5_SCAN_RIGHT_DEG +
                     S5_COARSE_STEP_DEG / 2) / S5_COARSE_STEP_DEG;
    if (index >= S5_COARSE_POINTS) {
        index = S5_COARSE_POINTS - 1u;
    }

    s_scan_samples[index].angle_deg = angle_deg;
    s_scan_samples[index].distance_cm = distance_cm;
    s_scan_samples[index].valid = valid;
    s_scan_count++;
}

ER subsystem5_init(void)
{
    clear_scan_samples();
    return (ultrasonic_init() == US_OK) ? E_OK : E_SYS;
}

ER subsystem5_start(void)
{
    ID task_id = app_start_task((FP)subsystem5_scan_task,
                                S5_PRI_SCAN, 0u, "s5_scan");
    return (task_id < E_OK) ? (ER)task_id : E_OK;
}

bool subsystem5_measure_at(int16_t angle_deg, s5_sample_t *out)
{
    size_t index;

    if (out == NULL || angle_deg < S5_SCAN_RIGHT_DEG ||
        angle_deg > S5_SCAN_LEFT_DEG) {
        return false;
    }

    index = (size_t)(angle_deg - S5_SCAN_RIGHT_DEG +
                     S5_COARSE_STEP_DEG / 2) / S5_COARSE_STEP_DEG;
    if (index >= S5_COARSE_POINTS) {
        index = S5_COARSE_POINTS - 1u;
    }
    *out = s_scan_samples[index];
    return out->valid;
}

bool subsystem5_check_ahead(float *distance_cm)
{
    size_t center_index =
        (S5_ANGLE_CENTER_DEG - S5_SCAN_RIGHT_DEG) / S5_COARSE_STEP_DEG;
    const s5_sample_t *center = &s_scan_samples[center_index];

    if (distance_cm != NULL) {
        *distance_cm = center->distance_cm;
    }
    return center->valid &&
        center->distance_cm <= S5_OBSTACLE_THRESHOLD_CM;
}

size_t subsystem5_coarse_scan(s5_sample_t *buf, size_t buf_len)
{
    size_t count = (buf_len < S5_COARSE_POINTS) ? buf_len :
                                                   S5_COARSE_POINTS;

    if (buf == NULL) {
        return 0u;
    }
    for (size_t i = 0; i < count; i++) {
        buf[i] = s_scan_samples[i];
    }
    return count;
}

static bool on_obstacle(const s5_sample_t *sample, float previous_cm)
{
    return sample->valid &&
        fabsf(sample->distance_cm - previous_cm) <= S5_EDGE_JUMP_CM;
}

static float lateral_cm(int16_t angle_deg, float distance_cm)
{
    return distance_cm *
        sinf(DEG2RAD(angle_deg - S5_ANGLE_CENTER_DEG));
}

static float forward_cm(int16_t angle_deg, float distance_cm)
{
    return distance_cm *
        cosf(DEG2RAD(angle_deg - S5_ANGLE_CENTER_DEG));
}

static float side_clearance(const s5_sample_t *samples, size_t count,
                            int16_t edge_deg, float edge_y, float center_cm,
                            int8_t direction)
{
    float best = US_MAX_RANGE_CM;

    for (size_t i = 0; i < count; i++) {
        const s5_sample_t *sample = &samples[i];
        bool beyond_edge;
        float gap;

        if (!sample->valid) {
            continue;
        }
        beyond_edge = (direction > 0)
            ? (sample->angle_deg > edge_deg)
            : (sample->angle_deg < edge_deg);
        if (!beyond_edge ||
            forward_cm(sample->angle_deg, sample->distance_cm) >
                center_cm + 40.0f) {
            continue;
        }

        gap = fabsf(lateral_cm(sample->angle_deg, sample->distance_cm)) -
            fabsf(edge_y);
        if (gap < 0.0f) {
            gap = 0.0f;
        }
        if (gap < best) {
            best = gap;
        }
    }
    return best;
}

bool subsystem5_profile_obstacle(s5_obstacle_profile_t *profile)
{
    s5_sample_t samples[S5_COARSE_POINTS];
    size_t center_index;
    size_t left_index;
    size_t right_index;
    float previous_cm;
    float left_y;
    float right_y;

    if (profile == NULL) {
        return false;
    }
    *profile = (s5_obstacle_profile_t){0};
    profile->direction = S5_DIR_NONE;

    if (!subsystem5_check_ahead(&profile->d_center_cm)) {
        return false;
    }
    profile->obstacle_detected = true;
    if (subsystem5_coarse_scan(samples, S5_COARSE_POINTS) !=
            S5_COARSE_POINTS) {
        return false;
    }

    center_index =
        (S5_ANGLE_CENTER_DEG - S5_SCAN_RIGHT_DEG) / S5_COARSE_STEP_DEG;
    left_index = center_index;
    previous_cm = profile->d_center_cm;
    while (left_index + 1u < S5_COARSE_POINTS &&
           on_obstacle(&samples[left_index + 1u], previous_cm)) {
        left_index++;
        previous_cm = samples[left_index].distance_cm;
    }

    right_index = center_index;
    previous_cm = profile->d_center_cm;
    while (right_index > 0u &&
           on_obstacle(&samples[right_index - 1u], previous_cm)) {
        right_index--;
        previous_cm = samples[right_index].distance_cm;
    }

    profile->left_edge_deg = samples[left_index].angle_deg;
    profile->d_left_cm = samples[left_index].distance_cm;
    profile->right_edge_deg = samples[right_index].angle_deg;
    profile->d_right_cm = samples[right_index].distance_cm;
    profile->left_edge_at_limit =
        (left_index == S5_COARSE_POINTS - 1u);
    profile->right_edge_at_limit = (right_index == 0u);

    left_y = lateral_cm(profile->left_edge_deg, profile->d_left_cm);
    right_y = lateral_cm(profile->right_edge_deg, profile->d_right_cm);
    profile->width_cm = left_y - right_y;
    profile->left_clearance_cm = side_clearance(
        samples, S5_COARSE_POINTS, profile->left_edge_deg, left_y,
        profile->d_center_cm, 1);
    profile->right_clearance_cm = side_clearance(
        samples, S5_COARSE_POINTS, profile->right_edge_deg, right_y,
        profile->d_center_cm, -1);

    if (profile->left_clearance_cm < S5_MIN_CLEARANCE_CM &&
        profile->right_clearance_cm < S5_MIN_CLEARANCE_CM) {
        profile->direction = S5_DIR_BLOCKED;
    } else if (profile->left_clearance_cm >= profile->right_clearance_cm) {
        profile->direction = S5_DIR_LEFT;
    } else {
        profile->direction = S5_DIR_RIGHT;
    }
    return true;
}

const char *subsystem5_direction_str(s5_direction_t direction)
{
    switch (direction) {
        case S5_DIR_NONE:    return "NONE";
        case S5_DIR_LEFT:    return "LEFT";
        case S5_DIR_RIGHT:   return "RIGHT";
        case S5_DIR_BLOCKED: return "BLOCKED";
        default:             return "UNKNOWN";
    }
}

void subsystem5_print_profile(const s5_obstacle_profile_t *profile)
{
    if (profile == NULL) {
        return;
    }

    tm_printf((UB*)"S5 obstacle dC=%dcm width=%dcm left=%dcm right=%dcm "
              "choose=%s\n",
              (INT)(profile->d_center_cm + 0.5f),
              (INT)(profile->width_cm + 0.5f),
              (INT)(profile->left_clearance_cm + 0.5f),
              (INT)(profile->right_clearance_cm + 0.5f),
              subsystem5_direction_str(profile->direction));
}

static void subsystem5_scan_task(INT stacd, void *exinf)
{
    T_CCYC cyclic = {0};
    ID cyclic_id;
    uint32_t previous_turn = 0u;
    int8_t previous_direction = 1;
    bool have_previous_ping = false;

    (void)stacd;
    (void)exinf;

    if (tk_dly_tsk(S5_STARTUP_DELAY_MS) < E_OK ||
        s5_servo_init() < E_OK ||
        s5_servo_move_to(S5_SCAN_RIGHT_DEG) < E_OK ||
        tk_dly_tsk(S5_SETTLE_MS) < E_OK) {
        tm_printf((UB*)"S5 servo initialization failed\n");
        tk_ext_tsk();
    }

    s_sweep_direction = 1;
    s_sweep_turns = 0u;
    s_servo_fault = false;
    cyclic.cycatr = TA_HLNG;
    cyclic.cychdr = (FP)s5_servo_step_handler;
    cyclic.cyctim = S5_SERVO_STEP_MS;
    cyclic.cycphs = S5_SERVO_STEP_MS;
    cyclic_id = tk_cre_cyc(&cyclic);
    if (cyclic_id < E_OK || tk_sta_cyc(cyclic_id) < E_OK) {
        tm_printf((UB*)"S5 servo scan timer initialization failed\n");
        tk_ext_tsk();
    }
    s_servo_scanning = true;

    for (;;) {
        float distance_cm;
        int16_t angle_deg;
        int8_t direction;
        uint32_t sweep_turn;
        /* The servo cyclic handler keeps stepping during this echo wait. */
        us_status_t status = ultrasonic_read_cm(
            &distance_cm, &angle_deg, &direction, &sweep_turn);
        bool valid = (status == US_OK || status == US_OUT_OF_RANGE);

        if (s_servo_fault) {
            tm_printf((UB*)"S5 servo PWM update failed\n");
            break;
        }

        if (have_previous_ping && sweep_turn != previous_turn) {
            if (previous_direction > 0) {
                s5_obstacle_profile_t profile;
                if (subsystem5_profile_obstacle(&profile)) {
                    subsystem5_print_profile(&profile);
                }
            } else {
                clear_scan_samples();
            }
        }

        if (valid || status == US_ERR_NO_ECHO) {
            store_scan_sample(angle_deg, distance_cm, valid);
            tm_printf((UB*)"S5 scan angle=%d distance=%dcm status=%s\n",
                      (INT)angle_deg,
                      (INT)(distance_cm + 0.5f),
                      valid ? "OK" : "NO_ECHO");
        } else {
            tm_printf((UB*)"S5 ultrasonic error: %d\n", (INT)status);
        }

        previous_turn = sweep_turn;
        previous_direction = direction;
        have_previous_ping = true;
    }

    s_servo_scanning = false;
    (void)tk_stp_cyc(cyclic_id);
    (void)tk_del_cyc(cyclic_id);
    tk_ext_tsk();
}

void subsystem5_test(void)
{
    uint32_t initial_count = s_scan_count;
    uint32_t startup_ms = S5_STARTUP_DELAY_MS +
        (S5_ANGLE_CENTER_DEG - S5_SCAN_RIGHT_DEG) * S5_SERVO_STEP_MS +
        S5_SETTLE_MS + 100u;

    (void)tk_dly_tsk(startup_ms);
    if (s_scan_count > initial_count) {
        tm_printf((UB*)"[TEST] S5 servo/ultrasonic scan is active\n");
    } else {
        tm_printf((UB*)"[TEST] S5 scan has not produced a reading\n");
    }
}
