#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H
#include "../app.h"
#include <stdbool.h>
#include <stdint.h>
/* Public compact status: flags9 + rpm9*2 + pwm7*2 + progress13*2 = 67 bits. */
typedef struct { uint8_t bytes[9]; } S2_STATUS;
static inline unsigned s2_get_bits(const S2_STATUS *s,unsigned pos,unsigned n)
{
    unsigned v=0;
    for(unsigned i=0;i<n;i++) v|=((s->bytes[(pos+i)/8]>>((pos+i)%8))&1u)<<i;
    return v;
}
static inline void s2_put_bits(S2_STATUS *s,unsigned pos,unsigned n,unsigned v)
{
    for(unsigned i=0;i<n;i++) {
        unsigned k=pos+i;
        s->bytes[k/8]=(uint8_t)((s->bytes[k/8]&~(1u<<(k%8)))|(((v>>i)&1u)<<(k%8)));
    }
}
EXPORT ER subsystem2_init(void);
EXPORT ER subsystem2_start(void);
EXPORT void motion_control_task(INT stacd,void *exinf);
EXPORT void subsystem2_test(void);
/* Called by your common GPIO dispatcher when S2_OWNS_GPIO_CALLBACK=0. */
void subsystem2_encoder_gpio_irq(unsigned gpio,uint32_t events);
/* Task context ONLY. These enqueue requests; they never write motor outputs.
   TODO TEAM: only the Vehicle Controller should normally call movement APIs. */
bool motion_set_speed(unsigned level); /* 0..4, zero requests priority stop */
bool motion_increase_speed(void);
bool motion_decrease_speed(void);
bool motion_forward(void);
bool motion_backward(void);
bool motion_set_wheel_levels(unsigned left,unsigned right);
bool motion_move_forward_mm(unsigned mm); /* 1..2000; 0 no-op */
bool motion_move_backward_mm(unsigned mm);
bool motion_turn_left(unsigned steps); /* 1..8 = 45..360 degrees */
bool motion_turn_right(unsigned steps);
void motion_stop(void); /* High-priority latch, discards queued commands. */
bool motion_clear_faults(void); /* Queued explicit reset; no auto-resume. */
bool motion_get_status(S2_STATUS *out);
bool motion_ready(void);
bool motion_busy(void); /* Includes queued commands and pending stop. */
unsigned motion_faults(void);
/* Last dequeued request result for diagnostic integration. Not a per-client reply queue. */
bool motion_last_result(uint32_t *sequence,bool *accepted);
/* Diagnostic bookkeeping, not part of packed motion/status payloads. */
enum { S2_REPLY_NONE,S2_REPLY_OK,S2_REPLY_STALE,S2_REPLY_SETTLING,
       S2_REPLY_FAULT,S2_REPLY_BUSY,S2_REPLY_ZERO_SPEED,S2_REPLY_INVALID };
typedef struct { uint32_t sequence,age_ms;uint16_t command;uint8_t reason;bool accepted; } S2_REPLY;
bool motion_get_reply(S2_REPLY *out);
#endif
