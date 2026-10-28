#include "subsystem2.h"
EXPORT void subsystem2_test(void)
{
    S2_STATUS s;
    if(motion_get_status(&s))tm_printf((UB*)"S2 mode=%u faults=%u rpm L=%u R=%u\n",
      s2_get_bits(&s,0,2),s2_get_bits(&s,5,4),s2_get_bits(&s,9,9),s2_get_bits(&s,18,9));
}
