/* RP2040 BSP adapter. No Pico SDK runtime, GPIO callbacks or hardware spinlocks.
 * Owner task and encoder interrupts MUST run on physical core 1 (TP_PRC2).
 * Register addresses come from RP2040 IO_BANK0/PWM/TIMER; other subsystems own
 * other peripheral registers. Default IRQ13 registration is for this isolated image. */
#include "motion_hw.h"
#include "motion_config.h"
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define S2_IO_INTR0 (IO_BANK0_BASE+0xf0u)
#define S2_PROC0_INTE0 (IO_BANK0_BASE+0x100u)
#define S2_PROC1_INTE0 (IO_BANK0_BASE+0x130u)
#define S2_PROC1_INTS0 (IO_BANK0_BASE+0x150u)
#define S2_IO_IRQ 13u
#define S2_EDGE_MASK ((8u<<(LEFT_ENC_A*4u))|(8u<<(RIGHT_ENC_A*4u)))
_Static_assert(LEFT_ENC_A<8 && RIGHT_ENC_A<8,"Default dispatcher supports GPIO0..7");
void s2_hw_safe_outputs(void)
{
    const unsigned pins[]={LEFT_IN_A,LEFT_IN_B,RIGHT_IN_A,RIGHT_IN_B};
    for(unsigned i=0;i<4;i++){
        gpio_set_val(pins[i],0);gpio_set_pin(pins[i],GPIO_MODE_OUT);
    }
}
void s2_hw_pwm_write(unsigned pin,unsigned level) { (void)pwm_set_cc(pin,level); }
void s2_hw_pwm_init(void)
{
    /* This kernel leaves PWM/TIMER in reset when USE_PTMR=0. Release only
       these blocks; do not reset unrelated devices or change system clocks. */
    REG32(RESETS_RESET+0x3000u)=RESETS_RESET_PWM|RESETS_RESET_TIMER;
    while((REG32(RESETS_RESET_DONE)&(RESETS_RESET_PWM|RESETS_RESET_TIMER))
          !=(RESETS_RESET_PWM|RESETS_RESET_TIMER)){}
    /* RP2040 timer is clocked from watchdog's 1us tick, not SysTick.
       Kernel SysTick remains unchanged. 12MHz clk_ref / 12 = 1MHz. */
    /* Do not trust a pre-enabled bootloader/debugger tick to have the right
       divider. Set the board's 12MHz/12 tick explicitly on each startup. */
    REG32(0x4005802cu)=0x20cu;
    REG32(TIMER_BASE+0x30u)&=~1u; /* Clear timer PAUSE, without resetting its count. */

    const unsigned pins[]={LEFT_IN_A,LEFT_IN_B,RIGHT_IN_A,RIGHT_IN_B};
    /* Kernel starts clk_sys at 125MHz. 125MHz/(6.25*1000)=20kHz.
       TODO TEAM: change S2_PWM_DIV16 if the board's system clock changes. */
    for(unsigned i=0;i<4;i++){
        unsigned slice=(pins[i]>>1)&7u;
        (void)pwm_set_enabled(pins[i],FALSE);
        REG32(PWM_BASE+slice*0x14u+0x04u)=S2_PWM_DIV16;
        REG32(PWM_BASE+slice*0x14u+0x08u)=0;
        (void)pwm_set_wrap(pins[i],999);
        (void)pwm_set_cc(pins[i],0);
        (void)pwm_set_pin(pins[i]);
    }
    (void)pwm_set_enabled(LEFT_IN_A,TRUE);
    (void)pwm_set_enabled(RIGHT_IN_A,TRUE);
}
uint64_t s2_hw_time_us(void)
{
    uint32_t hi,lo,again;
    do {hi=REG32((TIMER_BASE+0x24u));lo=REG32((TIMER_BASE+0x28u));again=REG32((TIMER_BASE+0x24u));}while(hi!=again);
    return ((uint64_t)hi<<32)|lo;
}
void s2_encoder_dispatch(void)
{
    /* Called only on core 1. Clear just our latched edge bits, not any teammate's. */
    uint32_t pending=REG32(S2_PROC1_INTS0)&S2_EDGE_MASK;
    REG32(S2_IO_INTR0)=pending;
    if(pending&(8u<<(LEFT_ENC_A*4u)))subsystem2_encoder_gpio_irq(LEFT_ENC_A,S2_GPIO_EDGE_RISE);
    if(pending&(8u<<(RIGHT_ENC_A*4u)))subsystem2_encoder_gpio_irq(RIGHT_ENC_A,S2_GPIO_EDGE_RISE);
}
#if S2_OWNS_GPIO_CALLBACK
static void io_bank0_handler(UINT intno) {(void)intno;s2_encoder_dispatch();}
#endif
ER s2_hw_encoder_init(void)
{
#if TK_SUPPORT_SMP
    if(tk_get_prc()!=2)return E_PAR;
#endif
    const unsigned pins[]={LEFT_ENC_A,LEFT_ENC_B,RIGHT_ENC_A,RIGHT_ENC_B};
    for(unsigned i=0;i<4;i++){
        gpio_set_pin(pins[i],GPIO_MODE_IN);
        REG32(GPIO(pins[i]))=(REG32(GPIO(pins[i]))&~GPIO_PDE)|GPIO_PUE;
    }
    /* Disable these edges on core 0. Only core 1 may touch the compact counter. */
    REG32(S2_PROC0_INTE0)&=~S2_EDGE_MASK;
    REG32(S2_PROC1_INTE0)&=~S2_EDGE_MASK;
#if S2_OWNS_GPIO_CALLBACK
    /* TODO TEAM: set S2_OWNS_GPIO_CALLBACK=0 and install a shared dispatcher instead.
       Do NOT register another IRQ13 handler which replaces this one. */
    T_DINT d={0};d.intatr=TA_HLNG;d.inthdr=(FP)io_bank0_handler;
    ER er=tk_def_int(S2_IO_IRQ,&d);if(er<E_OK)return er;
#endif
    REG32(S2_IO_INTR0)=S2_EDGE_MASK;
    REG32(S2_PROC1_INTE0)|=S2_EDGE_MASK;
#if S2_OWNS_GPIO_CALLBACK
    /* RP2040 IO_IRQ_BANK0 is IRQ13, vector29. Set lowest hardware priority. */
    REG32(0xe000e40cu)=(REG32(0xe000e40cu)&~0x0000ff00u)|0x0000c000u;
    REG32(0xe000e280u)=1u<<S2_IO_IRQ;
    REG32(0xe000e100u)=1u<<S2_IO_IRQ;
#endif
    return E_OK;
}
