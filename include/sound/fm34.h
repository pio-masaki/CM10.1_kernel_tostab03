#ifndef _FM34_H
#define _FM34_H

struct fm34_conf {
	int pwdn;		/* Fm34 PWDN#, power down */
	int rst;		/* Fm34 RST#, reset */
	int bp;			/* Fm34 PD#, hardware bypass */
	int cprop;	/* property array count */
	unsigned short *pprop;	/* property array */
};

extern bool set_fM34_bypass(int expect);
extern bool set_fM34_echo(void);
extern bool get_fM34_status(void);
extern void fm34_set_codec_link(void *codec, void (*codec_gain)(void *, int ));

#endif
