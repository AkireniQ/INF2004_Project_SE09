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