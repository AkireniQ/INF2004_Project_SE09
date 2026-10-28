/*
 *	subsystem1.c
 *	Subsystem 1 -- communication bridge/task plus compatible LED heartbeat.
 *
 *	APIs shown: tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"
#include "s1_comm.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
#if TM_MQTT
    ER er = (ER)mqtt_bridge_init();
    if (er < E_OK) { return er; }
#endif

	/* Cyclic handler: once started, the kernel calls led_cyclic_handler
	   every S1_TICK_MS.  Without TA_STA it is created stopped; it is
	   started in subsystem1_start(), once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;	/* period, ms */
	ccyc.cycphs = S1_TICK_MS;	/* delay before the first call, ms */
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_led = app_start_task((FP)led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	#if TM_MQTT
    {
        ID tid = app_start_task((FP)s1_comm_task, 13, 0, "s1_comm");
        if (tid < E_OK) { return tid; }
    }
#endif
    return tk_sta_cyc(cyc_led);	/* start the cyclic handler now */
}
