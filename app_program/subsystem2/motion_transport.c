#include "motion_transport.h"
#include "motion_config.h"


_Static_assert(S2_QUEUE_LENGTH==8,"Queue index bit widths assume length 8");
static ID transport_lock;
/* Native micro T-Kernel task-context mutex with priority inheritance.
 * Never call any transport API from an interrupt. Initialise ONCE at startup. */
static void lock_transport(void)
{
    ER er=tk_loc_mtx(transport_lock,TMO_FEVR);
    if(er<E_OK) {tm_printf((UB*)"S2 mutex lock failed: %d\n",er);for(;;)tk_slp_tsk(TMO_FEVR);}
}
static void unlock_transport(void)
{
    ER er=tk_unl_mtx(transport_lock);
    if(er<E_OK) {tm_printf((UB*)"S2 mutex unlock failed: %d\n",er);for(;;)tk_slp_tsk(TMO_FEVR);}
}
/* Four opcode bits and eleven union-argument bits, one unused container bit.
   Each arg variant only uses its required low bits: speed3,wheels6,angle3,distance11. */
static uint16_t commands[8];
/* Native RTOS timestamps are transport bookkeeping, not motion payload fields. */
static uint32_t issued_ms[8];
static struct {unsigned head:3;unsigned tail:3;unsigned count:4;unsigned stop:1;unsigned ready:1;unsigned result:1;unsigned processing:1;unsigned zero_level:1;} q;
static S2_STATUS cached;
static uint32_t result_seq;
static S2_REPLY reply;
uint32_t s2_now_ms(void) {SYSTIM t;tk_get_tim(&t);return t.lo;}
ER s2_transport_init(void)
{
    if(transport_lock>0)return E_OBJ;
    T_CMTX c={0};c.mtxatr=TA_INHERIT;
    ID id=tk_cre_mtx(&c);
    if(id<E_OK)return (ER)id;
    transport_lock=id;
    q=(__typeof__(q)){0};cached=(S2_STATUS){{0}};result_seq=0;reply=(S2_REPLY){0};
    return E_OK;
}
static bool submit(unsigned op,unsigned arg)
{
    if(!transport_lock) return false;
    uint32_t now=s2_now_ms();
    lock_transport();
    bool ok=q.ready && !q.stop && q.count<8;
    if(ok){commands[q.tail]=(uint16_t)(op | (arg<<4));issued_ms[q.tail]=now;q.tail=(q.tail+1)&7;q.count++;}
    unlock_transport();return ok;
}
static void request_stop(bool zero)
{
    if(!transport_lock)return;
    lock_transport();
    q.stop=1;q.zero_level |= zero;q.count=0;q.head=q.tail=0;
    unlock_transport();
}
void motion_stop(void){request_stop(false);}
bool motion_set_speed(unsigned n){if(n>4)return false;if(!n){if(!motion_ready())return false;request_stop(true);return true;}return submit(S2_SPEED,n);}
bool motion_increase_speed(void){return submit(S2_INC,0);}
bool motion_decrease_speed(void){return submit(S2_DEC,0);}
bool motion_forward(void){return submit(S2_FORWARD,0);}
bool motion_backward(void){return submit(S2_BACKWARD,0);}
bool motion_set_wheel_levels(unsigned l,unsigned r){if(l>4||r>4)return false;if(!l&&!r)return motion_set_speed(0);return submit(S2_WHEELS,l|(r<<3));}
bool motion_move_forward_mm(unsigned n){return n<=MAX_DISTANCE_MM && (n?submit(S2_MOVE_F,n):motion_ready());}
bool motion_move_backward_mm(unsigned n){return n<=MAX_DISTANCE_MM && (n?submit(S2_MOVE_B,n):motion_ready());}
bool motion_turn_left(unsigned n){return n>=1&&n<=8&&submit(S2_LEFT,n-1);}
bool motion_turn_right(unsigned n){return n>=1&&n<=8&&submit(S2_RIGHT,n-1);}
bool motion_clear_faults(void){return submit(S2_CLEAR,0);}
bool motion_get_status(S2_STATUS *out)
{
    if(!transport_lock||!out)return false;
    lock_transport();*out=cached;bool ready=q.ready;unlock_transport();return ready;
}
bool motion_ready(void){S2_STATUS s;return motion_get_status(&s);}
unsigned motion_faults(void){S2_STATUS s;if(!motion_get_status(&s))return 0;return s2_get_bits(&s,5,4);}
bool motion_busy(void)
{
    if(!transport_lock)return false;
    lock_transport();
    bool busy=q.count||q.stop||q.processing||s2_get_bits(&cached,0,2)!=0;
    unlock_transport();return busy;
}
bool motion_last_result(uint32_t *seq,bool *accepted)
{
    if(!transport_lock||!seq||!accepted)return false;
    lock_transport();*seq=result_seq;*accepted=q.result;bool ready=q.ready;
    unlock_transport();return ready;
}
bool s2_take_stop(bool *zero)
{
    lock_transport();bool stop=q.stop;*zero=q.zero_level;q.zero_level=0;q.stop=0;
    if(stop){q.count=0;q.head=q.tail=0;}unlock_transport();return stop;
}
bool s2_take_request(uint16_t *word,uint32_t *issued)
{
    lock_transport();bool found=q.count>0 && !q.stop;
    if(found){q.processing=1;*word=commands[q.head];*issued=issued_ms[q.head];q.head=(q.head+1)&7;q.count--;}
    unlock_transport();return found;
}
void s2_publish(const S2_STATUS *s,bool ready)
{lock_transport();cached=*s;q.ready=ready;unlock_transport();}
void s2_record_result(bool ok)
{lock_transport();q.result=ok;q.processing=0;result_seq++;unlock_transport();}
void s2_discard_queue(void)
{lock_transport();q.count=0;q.head=q.tail=0;unlock_transport();}

void s2_record_owner_result(uint16_t word,uint32_t age,unsigned reason,bool ok)
{
    lock_transport();
    q.result=ok;q.processing=0;result_seq++;
    reply=(S2_REPLY){.sequence=result_seq,.age_ms=age,.command=word,
                     .reason=(uint8_t)reason,.accepted=ok};
    unlock_transport();
}
bool motion_get_reply(S2_REPLY *out)
{
    if(!transport_lock||!out)return false;
    lock_transport();*out=reply;bool ready=q.ready;unlock_transport();return ready;
}
