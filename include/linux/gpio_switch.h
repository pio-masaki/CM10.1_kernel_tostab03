#ifndef _GPIO_SWITCH_H
#define _GPIO_SWITCH_H

struct gpio_switch {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int type;		/* input event type (EV_KEY, EV_SW) */
	int debounce_interval;	/* debounce ticks interval in msecs */
};

struct gpio_sw_platform_data {
	struct gpio_switch *switches;
	int nswitches;
};

#endif
