/* Binary v2 wire format. Bit 0 is the LSB of byte 0; no native bit-field ABI. */
#ifndef S1_WIRE_H
#define S1_WIRE_H
#include "../subsystem3/barcode.h"
#define S1_CMD_STOP 0u
#define S1_CMD_START 1u
#define S1_CMD_RESET 2u
#define S1_FAULT_MOTOR 1u
#define S1_FAULT_ENCODER 2u
#define S1_FAULT_IMU 4u
#define S1_FAULT_NO_ECHO 8u
#define S1_FAULT_WIFI 16u
#define S1_WIDTH_UNAVAILABLE 2047u
#define S1_CLEARANCE_OPEN 4094u
#define S1_CLEARANCE_UNAVAILABLE 4095u

/* v2 commands: one octet. 00 STOP, 01 START, 02 stopped-only RESET.
 * Exactly one byte; values above2 are invalid. No text payloads. */
int s1_command_decode(const unsigned char *data, unsigned length, unsigned *opcode);
/* Map packet: four 5-bit letters (L/R/straight/U-turn), then tag 0xA. */
#define S1_BARCODE_MAP_BYTES 3u
int s1_wire_barcode_map(unsigned char out[S1_BARCODE_MAP_BYTES], BARCODE_MAP map);
int s1_decode_barcode_map(const unsigned char *data, unsigned length, BARCODE_MAP *map);
void s1_wire_put(unsigned char *bytes, unsigned bit, unsigned count, unsigned value);
unsigned s1_wire_get(const unsigned char *bytes, unsigned bit, unsigned count);
int s1_wire_line(unsigned char out[1], unsigned ir, unsigned barcode);
int s1_wire_terrain(unsigned char out[3], unsigned height_mm, unsigned max_height_mm, unsigned phase);
int s1_wire_obstacle(unsigned char out[6], unsigned nearest_mm, unsigned width_mm,
                     unsigned left_mm, unsigned right_mm);
#endif
