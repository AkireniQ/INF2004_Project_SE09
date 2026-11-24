/* ir_sensor.c: reads the IR sensors through the template's ADC driver. */
#include "subsystem3.h"

LOCAL ID adc_dd = 0;

/* ch: 0 = GP26, 1 = GP27, 2 = GP28. Returns 0..4095, or -1 on error. */
EXPORT INT ir_read(INT ch)
{
    UW val = 0;
    SZ asz = 0;
    ER er;
    LOCAL BOOL shown = FALSE;      /* print each failure only once */

    if (adc_dd <= 0) {
        ID dd = tk_opn_dev((UB*)"adca", TD_READ);
        if (dd < E_OK) {
            if (!shown) { tm_printf((UB*)"[S3] tk_opn_dev(adca) = %d\n", dd); shown = TRUE; }
            return -1;
        }
        adc_dd = dd;
    }

    er = tk_srea_dev(adc_dd, ch, &val, 1, &asz);   /* synchronous read: waits for the result */
    if (er < E_OK || asz != 1) {
        if (!shown) { tm_printf((UB*)"[S3] tk_srea_dev ch%d = %d (asz=%d)\n", ch, er, (INT)asz); shown = TRUE; }
        return -1;
    }
    return (INT)(val & 0x0FFF);
}

/* ---- DO path for the barcode (optional, see S3_BC_DO_PIN) ----------------
 * Reads the module's comparator output directly: one bit, no ADC, no
 * calibration table.  Register addresses are RP2040 (Pico / Pico W).
 * Pico 2 (RP2350): IO_BANK0 = 0x40028000, PADS_BANK0 = 0x40038000, and
 * clear the pad ISO bit (bit 8) as well.  SIO is 0xD0000000 on both. */
#define S3_SIO_BASE        0xD0000000u
#define S3_SIO_GPIO_IN     (*(volatile UW *)(S3_SIO_BASE + 0x004))
#define S3_SIO_GPIO_OE_CLR (*(volatile UW *)(S3_SIO_BASE + 0x028))
#define S3_IO_BANK0_BASE   0x40014000u
#define S3_PADS_BANK0_BASE 0x4001C000u
#define S3_GPIO_CTRL(n)    (*(volatile UW *)(S3_IO_BANK0_BASE + 8u * (UW)(n) + 4u))
#define S3_PAD(n)          (*(volatile UW *)(S3_PADS_BANK0_BASE + 4u + 4u * (UW)(n)))

EXPORT BOOL ir_read_do(INT gpio)
{
    LOCAL UW ready_mask = 0;                    /* one bit per GPIO already configured */
    if (gpio < 0 || gpio > 29) { return FALSE; }
    if (!(ready_mask & (1u << gpio))) {
        S3_SIO_GPIO_OE_CLR = 1u << gpio;        /* input */
        S3_PAD(gpio) = (S3_PAD(gpio) | (1u << 6)) & ~(1u << 7);   /* IE=1, OD=0 */
        S3_GPIO_CTRL(gpio) = 5;                 /* FUNCSEL = SIO */
        ready_mask |= 1u << gpio;
    }
    return (S3_SIO_GPIO_IN >> gpio) & 1u;
}