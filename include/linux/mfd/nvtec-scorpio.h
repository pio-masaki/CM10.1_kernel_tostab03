#ifndef __LINUX_MFD_NVTEC_H
#define __LINUX_MFD_NVTEC_H

#include <linux/notifier.h>

struct nvtec_subdev_info {
    int id;
    const char *name;
    void  *platform_data;
};

struct nvtec_platform_data {
    int num_subdevs;
    struct nvtec_subdev_info *subdevs;
    /* Daniel Wang add for scorpio >>> */
    int request_pin_1;
    int request_pin_2;
    /* Daniel Wang add for scorpio <<< */
    int ap_wake_pin;
};

struct nvtec_battery_platform_data {
    unsigned int ac_in_pin;
    unsigned int batt_low_pin;
};

enum nvtec_event_type {
    NVTEC_EVENT_UNDOCK = 0x86,
    NVTEC_EVENT_SYSTEM = 0xC5,
    NVTEC_EVENT_BATTERY = 0xA8,
    NVTEC_EVENT_CIR = 0xCE,
    NVTEC_EVENT_CIR_REPEAT = 0xCF,
    NVTEC_FLASH_ACK = 0xFA,
    NVTEC_FLASH_NACK = 0xFE,
};

extern int nvtec_read_byte(struct device *dev, int reg, uint8_t *val);
extern int nvtec_read_word(struct device *dev, int reg, uint16_t *val);
extern int nvtec_read_block(struct device *dev, int reg, int len, uint8_t *val);
extern int nvtec_write_byte(struct device *dev, int reg, uint8_t val);
extern int nvtec_write_word(struct device *dev, int reg, uint16_t val);
extern int nvtec_write_block(struct device *dev, int reg, int len, uint8_t *val);

extern int nvtec_register_event_notifier(struct device *dev, 
                                         enum nvtec_event_type event,
                                         struct notifier_block *nb);
extern int nvtec_unregister_event_notifier(struct device *dev,
                                           enum nvtec_event_type event,
                                           struct notifier_block *nb);

extern int nvtec_prepare_to_power_off(void);

#endif  /* __LINUX_MFD_ */
