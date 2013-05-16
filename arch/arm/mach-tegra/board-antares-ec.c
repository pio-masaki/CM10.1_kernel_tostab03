#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/resource.h>

#include <linux/io.h>
#include <linux/mfd/nvtec-antares.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include "gpio-names.h"
#include "board.h"

#define EC_REQUEST_GPIO    TEGRA_GPIO_PBB1
#define AP_WAKE_GPIO       TEGRA_GPIO_PA0
#define AC_IN_GPIO         TEGRA_GPIO_PV3
#define BATT_LOW_GPIO      TEGRA_GPIO_PW3

static struct nvtec_battery_platform_data nvtec_battery_pdata = {
    .ac_in_pin = AC_IN_GPIO,
    .batt_low_pin = BATT_LOW_GPIO,
};

static struct nvtec_subdev_info nvtec_devs[] = {
    {
        .id = 0,
        .name = "nvtec-battery",
        .platform_data = &nvtec_battery_pdata,
    },
    {
        .id = 0,
        .name = "nvtec-cir",
        .platform_data = NULL,
    },
    {
        .id = 0,
        .name = "nvtec-undock",
        .platform_data = NULL,
    },

};

static struct nvtec_platform_data nvtec_pdata = {
    .num_subdevs = ARRAY_SIZE(nvtec_devs),
    .subdevs = nvtec_devs,
    .request_pin = EC_REQUEST_GPIO,
    .ap_wake_pin = AP_WAKE_GPIO,
};

static struct i2c_board_info __initdata antares_ec[] = {
    {
        I2C_BOARD_INFO("nvtec", 0x1b),
        .irq         = TEGRA_GPIO_TO_IRQ(AP_WAKE_GPIO),
        .platform_data = &nvtec_pdata,
    },
};

int __init antares_ec_init(void)
{
    tegra_gpio_enable(EC_REQUEST_GPIO);
    gpio_request(EC_REQUEST_GPIO, "ec_request");
    gpio_direction_output(EC_REQUEST_GPIO, 1);

    tegra_gpio_enable(AP_WAKE_GPIO);
    gpio_request(AP_WAKE_GPIO, "ap_wake");
    gpio_direction_input(AP_WAKE_GPIO);

    tegra_gpio_enable(AC_IN_GPIO);
    gpio_request(AC_IN_GPIO, "ac_in");
    gpio_direction_input(AC_IN_GPIO);
    
    tegra_gpio_enable(BATT_LOW_GPIO);
    gpio_request(BATT_LOW_GPIO, "batt_low");
    gpio_direction_input(BATT_LOW_GPIO);

    i2c_register_board_info(3, antares_ec, 1);
    return 0;
}
