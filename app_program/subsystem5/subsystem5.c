LOCAL volatile UW s5_faults;    /* shared bitmask */

LOCAL void s5_set_fault(UW bit)
{
	UINT intsts;
	DI(intsts);          /* protect the read-modify-write */
	s5_faults |= bit;
	EI(intsts);
}

LOCAL void s5_task(INT stacd, void *exinf)
{
	UW cm, profile; INT r;
	drv_us_init(S5_TRIG_PIN, S5_ECHO_PIN);

	while (1) {
		r = drv_us_read_cm(S5_TRIG_PIN, S5_ECHO_PIN, &cm);
		if (r == US_OK) {
			profile = S5_PACK(cm, 0, 0, 0, 1);   /* angle/width/clearance later */
			tm_printf((UB*)"S5 profile=0x%08x dist=%d cm\n", profile, (INT)S5_DIST(profile));
		} else {
			s5_set_fault(r == US_ERR_NOSTART ? S5_FAULT_NOECHO : S5_FAULT_STUCK);
			tm_printf((UB*)"S5 fault, mask=0x%x\n", (UINT)s5_faults);
		}
		tk_dly_tsk(100);
	}
}