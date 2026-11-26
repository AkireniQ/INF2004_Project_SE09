#include "s5_profile.h"

#define MAX_WIDTH_MM 2000
#define MAX_EDGE_BRACKET_STEPS 2u
#define MIN_DISTANT_ECHOES 3u
#define WIDTH_COS_SCALE 65536u

_Static_assert(S5_FOLLOW_CLEAR_MM <= S5_MAX_PROJECTED_FRONT_MM, "Front projection must cover the obstacle window");

/* sin and cos for 0..75 degrees in steps of 5 degrees, scaled by 1024. */
static const unsigned short sine[] = {
    0, 89, 178, 265, 350, 433, 512, 587, 658, 724, 784, 839, 887, 928, 962, 989,
};
static const unsigned short cosine[] = {
    1024, 1020, 1008, 989, 962, 928, 886, 838, 784, 724, 658, 587, 512, 433, 350, 265,
};

/* 1 - cos(angle) for edge separations of 0..150 degrees in steps of 5 degrees, scaled by 65536. */
static const uint32_t one_minus_cosine[] = {
    0, 249, 996, 2233, 3952,
    6140, 8780, 11852, 15333, 19195,
    23410, 27946, 32768, 37839, 43121,
    48574, 54156, 59824, 65536, 71248,
    76916, 82498, 87951, 93233, 98304,
    103126, 107662, 111877, 115739, 119220,
    122292,
};
_Static_assert(S5_FINE_STEP == 5u && S5_SCAN_POINTS <= sizeof(one_minus_cosine) / sizeof(*one_minus_cosine),
               "The width cosine table must cover the configured sweep");

static unsigned angle_steps(unsigned index)
{
    return index < S5_CENTRE_INDEX ? S5_CENTRE_INDEX - index : index - S5_CENTRE_INDEX;
}

static int lateral_mm(unsigned index, unsigned range_mm)
{
    int projection = range_mm * sine[angle_steps(index)] / 1024u;
    return index < S5_CENTRE_INDEX ? -projection : projection;
}

static unsigned project_forward_mm(unsigned index, unsigned range_mm)
{
    return range_mm * cosine[angle_steps(index)] / 1024u;
}

static int edge_bound_mm(unsigned bearing, unsigned obstacle_depth)
{
    unsigned angle = angle_steps(bearing);
    int projection = (obstacle_depth * sine[angle] + cosine[angle] - 1u) / cosine[angle];
    return bearing < S5_CENTRE_INDEX ? -projection : projection;
}

void s5_scan_set(S5_SCAN *scan, unsigned index, unsigned range)
{
    unsigned offset = index * 3u / 2u;
    if (index & 1u) {
        scan->bytes[offset] = (scan->bytes[offset] & 15u) | ((range & 15u) << 4u);
        scan->bytes[offset + 1u] = range >> 4u;
    } else {
        scan->bytes[offset] = range & 255u;
        scan->bytes[offset + 1u] = (scan->bytes[offset + 1u] & 240u) | (range >> 8u);
    }
}

unsigned s5_scan_get(const S5_SCAN *scan, unsigned index)
{
    unsigned offset = index * 3u / 2u;
    unsigned word = scan->bytes[offset] | ((unsigned)scan->bytes[offset + 1u] << 8u);
    return (word >> ((index & 1u) * 4u)) & 4095u;
}

unsigned s5_scan_target(const S5_SCAN *scan, unsigned limit_mm)
{
    /* Give priority to a return directly in the vehicle's path. */
    if (s5_scan_get(scan, S5_CENTRE_INDEX) <= limit_mm) return S5_CENTRE_INDEX;
    unsigned target = S5_NO_TARGET;
    unsigned closest = limit_mm + 1;
    for (unsigned i = S5_COARSE_FIRST; i <= S5_COARSE_LAST; i += S5_COARSE_STEP) {
        unsigned range = s5_scan_get(scan, i);
        if (range < closest) {
            target = i;
            closest = range;
        }
    }
    return target;
}

bool s5_same_obstacle(unsigned range, unsigned reference)
{
    return s5_range_measured(range) && s5_range_measured(reference) &&
           range <= reference + S5_PROFILE_TOLERANCE_MM &&
           reference <= range + S5_PROFILE_TOLERANCE_MM;
}

bool s5_background_beyond(unsigned range, unsigned reference)
{
    return range == S5_OPEN_RANGE ||
           (s5_range_measured(range) && range > reference + S5_PROFILE_TOLERANCE_MM);
}

unsigned s5_edge_background(const unsigned *ranges, unsigned count, unsigned reference)
{
    unsigned shortest = S5_UNKNOWN_RANGE;
    unsigned previous = S5_UNKNOWN_RANGE;
    unsigned distant_echoes = 0;
    bool repeated = false;
    for (unsigned i = 0; i < count; ++i) {
        unsigned range = ranges[i];
        if (s5_range_measured(range)) {
            /* A single near return leaves the bearing in the edge bracket. */
            if (!s5_background_beyond(range, reference)) return S5_UNKNOWN_RANGE;
            ++distant_echoes;
        }
        if (s5_range_available(s5_confirm_scan_range(previous, range))) repeated = true;
        if (s5_range_available(range) && range < shortest) shortest = range;
        if (s5_range_available(range) || !s5_range_measured(previous)) previous = range;
    }
    /* Beyond the edge, separate reflectors can produce different distances. */
    return repeated || distant_echoes >= MIN_DISTANT_ECHOES ? shortest : S5_UNKNOWN_RANGE;
}

unsigned s5_clearance_range(const unsigned *ranges, unsigned count)
{
    unsigned nearest = S5_UNKNOWN_RANGE;
    unsigned measured = 0;
    unsigned open = 0;
    for (unsigned i = 0; i < count; ++i) {
        unsigned range = ranges[i];
        if (s5_range_measured(range)) {
            ++measured;
            if (range < nearest) nearest = range;
        } else if (range == S5_OPEN_RANGE) {
            ++open;
        }
    }
    /* A corridor can return echoes from different surfaces. Use the nearest
       measured distance when two usable returns support the observation. */
    if (measured + open < 2u) return S5_UNKNOWN_RANGE;
    return measured ? nearest : S5_OPEN_RANGE;
}

typedef struct {
    unsigned last_hit, boundary, clearance;
} S5_EDGE;

static unsigned width_between_echoes_mm(unsigned left_range, unsigned right_range, unsigned separation_steps)
{
    /* Law of cosines: W² = (d_left - d_right)² + 2 d_left d_right (1 - cos(delta)). */
    int range_difference = (int)left_range - (int)right_range;
    uint64_t width_squared = (uint64_t)(range_difference * range_difference) * WIDTH_COS_SCALE +
                             2u * (uint64_t)left_range * right_range * one_minus_cosine[separation_steps];
    unsigned square_mm = (unsigned)(width_squared / WIDTH_COS_SCALE);
    unsigned low = 0, high = left_range + right_range;
    while (low < high) {
        unsigned midpoint = (low + high + 1u) / 2u;
        if (midpoint * midpoint <= square_mm) low = midpoint;
        else high = midpoint - 1u;
    }
    /* Compare with (low + 0.5)^2 to round to the nearest millimetre. */
    uint64_t round_up_at = (uint64_t)low * (low + 1u) * WIDTH_COS_SCALE + WIDTH_COS_SCALE / 4u;
    return low + (width_squared >= round_up_at);
}

static S5_EDGE profile_edge(const S5_SCAN *scan, unsigned anchor, unsigned limit, int step)
{
    S5_EDGE edge = {.last_hit = anchor, .clearance = S5_UNKNOWN_RANGE};
    unsigned reference = s5_scan_get(scan, anchor);
    for (int i = (int)anchor; i != (int)limit;) {
        i += step;
        unsigned range = s5_scan_get(scan, (unsigned)i);
        if (!s5_range_available(range)) continue;
        if (s5_background_beyond(range, reference)) {
            edge.boundary = (unsigned)i;
            edge.clearance = range;
            break;
        }
        if (s5_same_obstacle(range, reference)) edge.last_hit = (unsigned)i;
    }
    return edge;
}

S5_PROFILE s5_profile(const S5_SCAN *scan, unsigned first, unsigned last, unsigned anchor,
                      unsigned *observed_span_mm)
{
    S5_EDGE left = profile_edge(scan, anchor, first, -1);
    S5_EDGE right = profile_edge(scan, anchor, last, 1);
    unsigned near = anchor;
    unsigned reference = s5_scan_get(scan, anchor);
    unsigned front = reference;
    int left_hit = lateral_mm(anchor, reference);
    int right_hit = left_hit;
    for (unsigned i = left.last_hit; i <= right.last_hit; ++i) {
        unsigned range = s5_scan_get(scan, i);
        if (!s5_same_obstacle(range, reference)) continue;
        /* The legacy telemetry angle field represents the standard bearings. */
        unsigned bearing = s5_scan_bearing(i);
        if (bearing >= S5_MIN_ANGLE && bearing <= S5_MAX_ANGLE && range < s5_scan_get(scan, near))
            near = i;
        int lateral = lateral_mm(i, range);
        /* Off-path reflections at wide angles do not shorten the gap directly ahead. */
        if (lateral >= -(int)S5_TURN_CLEARANCE_MM && lateral <= (int)S5_TURN_CLEARANCE_MM) {
            unsigned forward = project_forward_mm(i, range);
            if (forward < front) front = forward;
        }
        if (lateral < left_hit) left_hit = lateral;
        if (lateral > right_hit) right_hit = lateral;
    }
    unsigned span = (unsigned)(right_hit - left_hit);
    if (observed_span_mm) *observed_span_mm = span;
    S5_PROFILE result = {
        .nearest_mm = s5_scan_get(scan, near),
        .angle_offset_deg = S5_MAX_ANGLE - s5_scan_bearing(near),
        .front_mm = front < S5_MAX_PROJECTED_FRONT_MM ? front : S5_MAX_PROJECTED_FRONT_MM,
        .width_mm = span > 0 && span <= MAX_WIDTH_MM ? (span + 1u) / 2u : S5_UNKNOWN_WIDTH,
        .left_edge_mm = left_hit,
        .right_edge_mm = right_hit,
        .left_mm = left.clearance,
        .right_mm = right.clearance,
        .left_path_mm = S5_UNKNOWN_RANGE,
        .right_path_mm = S5_UNKNOWN_RANGE,
        .left_outward_mm = S5_UNKNOWN_RANGE,
        .right_outward_mm = S5_UNKNOWN_RANGE,
        .left_clipped = left.last_hit == 0,
        .right_clipped = right.last_hit == S5_SCAN_POINTS - 1,
    };
    /* Project the background bearing onto the depth of the adjacent obstacle face. */
    if (s5_range_available(left.clearance) && left.last_hit - left.boundary <= MAX_EDGE_BRACKET_STEPS) {
        unsigned depth = project_forward_mm(left.last_hit, s5_scan_get(scan, left.last_hit));
        result.left_edge_mm = edge_bound_mm(left.boundary, depth);
        result.left_edge_valid = true;
    }
    if (s5_range_available(right.clearance) && right.boundary - right.last_hit <= MAX_EDGE_BRACKET_STEPS) {
        unsigned depth = project_forward_mm(right.last_hit, s5_scan_get(scan, right.last_hit));
        result.right_edge_mm = edge_bound_mm(right.boundary, depth);
        result.right_edge_valid = true;
    }
    /* An incomplete sweep can still report the observed lateral span. */
    if (!result.left_edge_valid || !result.right_edge_valid) {
        result.width_status = result.left_edge_valid ? S5_WIDTH_RIGHT_EDGE :
                              result.right_edge_valid ? S5_WIDTH_LEFT_EDGE : S5_WIDTH_BOTH_EDGES;
    } else {
        /* The adjacent background returns confirm the edges. */
        unsigned width = width_between_echoes_mm(s5_scan_get(scan, left.last_hit),
                                                 s5_scan_get(scan, right.last_hit),
                                                 right.last_hit - left.last_hit);
        if (width > 0 && width <= MAX_WIDTH_MM) result.width_mm = (width + 1u) / 2u;
        else result.width_status = S5_WIDTH_UNRESOLVED;
    }
    if (result.width_mm != S5_UNKNOWN_WIDTH) {
        /* Apply the same beam-width correction to the planned edges as to the
           displayed width. Preserve the observed midpoint for off-centre boxes. */
        int midpoint = left_hit + (int)((span + 1u) / 2u);
        int left_estimate = midpoint - (int)((result.width_mm + 1u) / 2u);
        int right_estimate = left_estimate + (int)result.width_mm;
        if (result.left_edge_valid) result.left_edge_mm = left_estimate;
        if (result.right_edge_valid) result.right_edge_mm = right_estimate;
    }
    return result;
}
