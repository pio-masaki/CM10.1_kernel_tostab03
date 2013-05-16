#ifndef __MT9D115_H__
#define __MT9D115_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9D115_NAME    "mt9d115"
#define MT9D115_PATH    "/dev/mt9d115"

#define MT9D115_IOCTL_SET_MODE          _IOW('o', 1, struct mt9d115_mode)
#define MT9D115_IOCTL_SET_COLOR_EFFECT  _IOW('o', 2, unsigned int)
#define MT9D115_IOCTL_SET_WHITE_BALANCE _IOW('o', 3, unsigned int)
#define MT9D115_IOCTL_SET_EXPOSURE      _IOW('o', 4, int)
#define MT9D115_IOCTL_SET_EXPOSURE_RECT _IOW('o', 5, struct mt9d115_rect)
#define MT9D115_IOCTL_GET_ISO           _IOW('o', 6, unsigned int)
#define MT9D115_IOCTL_GET_EXPOSURE_TIME _IOW('o', 7, unsigned int)

struct mt9d115_mode {
	int xres;
	int yres;
};

struct mt9d115_rect{
    int x;
    int y;
    int width;
    int height;
};

#ifdef __KERNEL__

struct mt9d115_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};

void lock_cam_i2c(void);
void unlock_cam_i2c(void);

#endif /* __KERNEL__ */

#endif  /* __MT9D115_H__ */

