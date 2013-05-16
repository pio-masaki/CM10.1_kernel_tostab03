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
#define MT9P111_IOCTL_SET_FLASH_MODE    _IOW('o', 9, unsigned int)
#define MT9P111_IOCTL_CAPTURE_CMD    	_IOW('o', 10, unsigned int)
#define MT9P111_IOCTL_SET_BRIGHTNESS   	_IOW('o', 11, unsigned int)
#define MT9P111_IOCTL_SET_CONTRAST   	_IOW('o', 12, unsigned int)
#define MT9P111_IOCTL_SET_EXPOSURE_METERING _IOW('o', 13, unsigned int)
#define MT9P111_IOCTL_GET_FLASH_STATUS  _IOW('o', 14, unsigned int)
#define MT9P111_IOCTL_GET_ISO           _IOW('o', 15, unsigned int)
#define MT9P111_IOCTL_GET_EXPOSURE_TIME _IOW('o', 16, unsigned int)

struct mt9p111_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__

struct mt9p111_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	int (*torch_on)(void);
	int (*torch_off)(void);

};

#endif /* __KERNEL__ */

#endif  /* __MT9P111_H__ */

