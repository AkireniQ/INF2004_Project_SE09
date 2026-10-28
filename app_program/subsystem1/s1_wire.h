/* Binary v2 wire format. Bit 0 is the LSB of byte 0; no native bit-field ABI. */
#ifndef S1_WIRE_H
#define S1_WIRE_H
#define S1_CMD_STOP 0u
#define S1_CMD_START 1u
#define S1_FAULT_MOTOR 1u
#define S1_FAULT_ENCODER 2u
#define S1_FAULT_IMU 4u
#define S1_FAULT_NO_ECHO 8u
#define S1_FAULT_WIFI 16u

/* v2 commands: one application bit in one octet. 00 STOP, 01 START.
 * Exactly one byte, with bits 1..7 zero. No text payloads. */
int s1_command_decode(const unsigned char *data, unsigned length, unsigned *opcode);
void s1_wire_put(unsigned char *bytes, unsigned bit, unsigned count, unsigned value);
unsigned s1_wire_get(const unsigned char *bytes, unsigned bit, unsigned count);
int s1_wire_line(unsigned char out[1], unsigned ir, unsigned barcode);
int s1_wire_terrain(unsigned char out[2], unsigned height_mm, unsigned phase);
int s1_wire_obstacle(unsigned char out[6], unsigned nearest_mm, unsigned width_mm,
                     unsigned left_mm, unsigned right_mm);
#endif
