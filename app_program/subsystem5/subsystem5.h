#define S5_TRIG_PIN 6
#define S5_ECHO_PIN 7

/* Bit layout of the 32-bit profile word. */
#define S5_DIST_POS 0   /* 9 bits: 0-400 cm   */
#define S5_ANG_POS  9   /* 6 bits: angle index*/
#define S5_WID_POS  15  /* 6 bits: width cm   */
#define S5_CLR_POS  21  /* 6 bits: clearance  */
#define S5_STATE_POS 27 /* 3 bits: scan state */

#define S5_PACK(d,a,w,c,s) ( ((UW)(d) & 0x1FF) << S5_DIST_POS | \
	((UW)(a) & 0x3F) << S5_ANG_POS | ((UW)(w) & 0x3F) << S5_WID_POS | \
	((UW)(c) & 0x3F) << S5_CLR_POS | ((UW)(s) & 0x7) << S5_STATE_POS )
#define S5_DIST(p) (((p) >> S5_DIST_POS) & 0x1FF)

#define S5_FAULT_NOECHO   (1U << 0)
#define S5_FAULT_STUCK    (1U << 1)
#define S5_FAULT_SERVO    (1U << 2)