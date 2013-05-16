#ifndef _ISL29018_DEV_H_
#define _ISL29018_DEV_H_

#define SET_ENABLE      0x00
#define GET_ENABLE      0x01
#define RD_LUX          0x02
#define RD_PROX         0x03

#define LIGHT_IOC_MAGIC     'v'
#define LIGHT_IOCRESET   _IO(LIGHT_IOC_MAGIC,0)

#define LIGHT_SET_ENABLE _IOW(LIGHT_IOC_MAGIC, SET_ENABLE, int)
#define LIGHT_GET_ENABLE _IOR(LIGHT_IOC_MAGIC, GET_ENABLE, int)
#define LIGHT_RD_LUX     _IOR(LIGHT_IOC_MAGIC, RD_LUX,     int)
#define LIGHT_RD_PROX    _IOR(LIGHT_IOC_MAGIC, RD_PROX,    int)

#endif //_ISL29018_DEV_H_