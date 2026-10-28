#ifndef MOTION_TRANSPORT_H
#define MOTION_TRANSPORT_H
#include "subsystem2.h"
enum {S2_SPEED,S2_INC,S2_DEC,S2_FORWARD,S2_BACKWARD,S2_WHEELS,S2_MOVE_F,S2_MOVE_B,S2_LEFT,S2_RIGHT,S2_CLEAR};
ER s2_transport_init(void);
uint32_t s2_now_ms(void);
bool s2_take_request(uint16_t *word,uint32_t *issued);
bool s2_take_stop(bool *zero);
void s2_publish(const S2_STATUS *status,bool ready);
void s2_record_result(bool accepted);
void s2_discard_queue(void);
void s2_record_owner_result(uint16_t word,uint32_t age,unsigned reason,bool accepted);
#endif
