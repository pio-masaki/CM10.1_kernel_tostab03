#ifndef __MT9P111_H__
#define __MT9P111_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9P111_NAME    "mt9p111"
#define MT9P111_PATH    "/dev/mt9p111"

#define MT9P111_IOCTL_SET_MODE          _IOW('o', 1, struct mt9p111_mode)
#define MT9P111_IOCTL_SET_COLOR_EFFECT  _IOW('o', 2, unsigned int)
#define MT9P111_IOCTL_SET_WHITE_BALANCE _IOW('o', 3, unsigned int)
#define MT9P111_IOCTL_SET_EXPOSURE      _IOW('o', 4, int)
#define MT9P111_IOCTL_SET_AF_MODE       _IOW('o', 5, unsigned int)
#define MT9P111_IOCTL_SET_AF_TRIGGER    _IOW('o', 6, unsigned int)
#define MT9P111_IOCTL_GET_AF_STATUS     _IOW('o', 7, unsigned int)
#define MT9P111_IOCTL_SET_FPS           _IOW('o', 8, unsigned int)
#define MT9P111_IOCTL_GET_ISO           _IOW('o', 9, unsigned int)
#define MT9P111_IOCTL_GET_EXPOSURE_TIME _IOW('o', 10, unsigned int)

struct mt9p111_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__

struct mt9p111_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};

#endif /* __KERNEL__ */

#endif  /* __MT9P111_H__ */

