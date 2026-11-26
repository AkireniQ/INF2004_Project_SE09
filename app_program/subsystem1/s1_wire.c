#include "s1_wire.h"
#include "../subsystem4/b4_terrain_report.h"

void s1_wire_put(unsigned char *bytes, unsigned bit, unsigned count, unsigned value)
{
    unsigned i;
    for (i = 0; i < count; ++i) {
        unsigned k = bit + i;
        bytes[k / 8u] = (unsigned char)((bytes[k / 8u] & ~(1u << (k % 8u))) |
                           (((value >> i) & 1u) << (k % 8u)));
    }
}

unsigned s1_wire_get(const unsigned char *bytes, unsigned bit, unsigned count)
{
    unsigned i;
    unsigned value = 0;
    for (i = 0; i < count; ++i) {
        unsigned k = bit + i;
        value |= ((unsigned)(bytes[k / 8u] >> (k % 8u)) & 1u) << i;
    }
    return value;
}

int s1_command_decode(const unsigned char *data, unsigned length, unsigned *opcode)
{
    if (data == 0 || opcode == 0 || length != 1u || data[0] > S1_CMD_RESET) { return 0; }
    *opcode = data[0];
    return 1;
}

int s1_wire_barcode_map(unsigned char out[S1_BARCODE_MAP_BYTES], BARCODE_MAP map)
{
    if (!out || !barcode_map_valid(map)) return 0;
    out[0] = out[1] = out[2] = 0;
    s1_wire_put(out, 0, 5, map.left);
    s1_wire_put(out, 5, 5, map.right);
    s1_wire_put(out, 10, 5, map.straight);
    s1_wire_put(out, 15, 5, map.uturn);
    s1_wire_put(out, 20, 4, 0xAu);
    return 1;
}

int s1_decode_barcode_map(const unsigned char *data, unsigned length, BARCODE_MAP *map)
{
    if (!data || !map || length != S1_BARCODE_MAP_BYTES || s1_wire_get(data, 20, 4) != 0xAu)
        return 0;
    BARCODE_MAP decoded = {
        .left = s1_wire_get(data, 0, 5), .right = s1_wire_get(data, 5, 5),
        .straight = s1_wire_get(data, 10, 5), .uturn = s1_wire_get(data, 15, 5),
    };
    if (!barcode_map_valid(decoded)) return 0;
    *map = decoded;
    return 1;
}

int s1_wire_line(unsigned char out[1], unsigned ir, unsigned barcode)
{
    if (out == 0 || ir > 7u || barcode > 26u) { return 0; }
    out[0] = (unsigned char)(ir | (barcode << 3u));
    return 1;
}

int s1_wire_terrain(unsigned char out[S1_TERRAIN_BYTES], unsigned height_mm)
{
    if (!out || height_mm > B4_TERRAIN_HEIGHT_MAX_MM) return 0;
    /* Reuse the two-byte layout with its phase set to unavailable. */
    out[0] = (unsigned char)height_mm;
    out[1] = (unsigned char)((height_mm >> 8) | (B4_TERRAIN_PHASE_UNAVAILABLE << 3));
    return 1;
}

int s1_wire_obstacle(unsigned char out[S1_OBSTACLE_BYTES], S1_OBSTACLE_STATUS status, unsigned width_mm)
{
    if (!out || (unsigned)status > S1_OBSTACLE_DETECTED ||
        (width_mm > 2000u && width_mm != S1_WIDTH_UNAVAILABLE)) return 0;
    out[0] = (unsigned char)width_mm;
    out[1] = (unsigned char)((width_mm >> 8) | ((unsigned)status << 3));
    return 1;
}
