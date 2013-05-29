#ifndef __NS9360FB_H__
#define __NS9360FB_H__


struct ns9360fb_display {

	/* Display name */
	char *display_name;		

	unsigned height;
	unsigned width;
	unsigned clock;

	u32 timing[4];
	u32 control;

	void (*display_power_enable)(int);
};

struct ns9360fb_pdata {
	unsigned num_displays;			/* number of defined displays */
	struct ns9360fb_display *displays;	/* attached diplays info */
	struct ns9360fb_display *display;	/* attached diplays info */
};

#endif
