#ifndef _NCT1008_DEV_H_
#define _NCT1008_DEV_H_

#include <linux/ioctl.h>

#define SET_ENABLE      0x00
#define GET_ENABLE      0x01
#define RD_TEMP         0x02

#define THOM_IOC_MAGIC     't'
#define THOM_IOCRESET   _IO(THOM_IOC_MAGIC,0)

#define THOM_SET_ENABLE _IOW(THOM_IOC_MAGIC, SET_ENABLE, int)
#define THOM_GET_ENABLE _IOR(THOM_IOC_MAGIC, GET_ENABLE, int)
#define THOM_RD_TEMP    _IOR(THOM_IOC_MAGIC, RD_TEMP,    int)

#endif //_NCT1008_DEV_H_
