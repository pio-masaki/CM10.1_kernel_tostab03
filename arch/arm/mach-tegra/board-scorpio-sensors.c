/*
 * arch/arm/mach-tegra/board-scorpio-sensors.c
 *
 * Copyright (c) 2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mpu.h>
#include <linux/i2c/pca954x.h>
#include <linux/i2c/pca953x.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#ifdef CONFIG_VIDEO_MT9P111_SCORPIO
#include <media/mt9p111-scorpio.h>
#endif
#ifdef CONFIG_VIDEO_MT9D115_SCORPIO
#include <media/mt9d115.h>
#endif
#include <media/ssl3250a.h>
#include <generated/mach-types.h>

#include "gpio-names.h"
#include "board.h"
#include "board-scorpio.h"
#include "cpu-tegra.h"

#define ISL29018_IRQ_GPIO	TEGRA_GPIO_PZ2
#define AKM8975_IRQ_GPIO	TEGRA_GPIO_PN5
#define CAMERA_POWER_GPIO	TEGRA_GPIO_PV4
#define CAMERA_CSI_MUX_SEL_GPIO	TEGRA_GPIO_PBB4
#define CAMERA_FLASH_ACT_GPIO	TEGRA_GPIO_PD2
#define NCT1008_THERM2_GPIO	TEGRA_GPIO_PN6
#define CAM_MTP9111_RST_GPIO    TEGRA_GPIO_PP3
#define CAM_MTP9111_PWD_GPIO    TEGRA_GPIO_PP2
#define CAM_MTP9115_RST_GPIO    TEGRA_GPIO_PP0
#define CAM_MTP9115_PWD_GPIO    TEGRA_GPIO_PP1



#define CAMERA_GPIO(_name, _gpio, _enabled, _milliseconds)      \
	{                                                       \
		.name = _name,                                  \
		.gpio = _gpio,                                  \
		.enabled = _enabled,                            \
		.milliseconds = _milliseconds,                  \
	}

struct camera_gpios {
	const char *name;
	int gpio;
	int enabled;
	int milliseconds;
};

extern void tegra_throttling_enable(bool enable);


static struct pca953x_platform_data antares_tca6416_data = {
	.gpio_base      = TEGRA_NR_GPIOS + 4, /* 4 gpios are already requested by tps6586x */
};

static const struct i2c_board_info antares_i2c3_board_info_tca6416[] = {
	{
		I2C_BOARD_INFO("tca6416", 0x20),
		.platform_data = &antares_tca6416_data,
	},
};


#ifdef CONFIG_VIDEO_MT9P111_SCORPIO
static const struct camera_gpios mt9p111_gpio_keys[] = {
	[0] = CAMERA_GPIO("en_avdd_csi", AVDD_DSI_CSI_ENB_GPIO, 1, 1),
};

static int antares_mt9p111_torch_on(void)
{
	gpio_direction_output(TEGRA_GPIO_PBB5, 1);
	return 0;
}


static int antares_mt9p111_torch_off(void)
{
	gpio_direction_output(TEGRA_GPIO_PBB5, 0);
	return 0;
}

static int antares_mt9p111_power_on(void)
{
    int ret = 0;
    gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
    mdelay(1);    
	gpio_direction_output(CAM_MTP9111_PWD_GPIO, 0);
    mdelay(1);    
    gpio_direction_output(CAM_MTP9111_RST_GPIO, 1);
    mdelay(1);    


	return ret;
}

static int antares_mt9p111_power_off(void)
{

    gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
    mdelay(1);
	gpio_direction_output(CAM_MTP9111_PWD_GPIO, 1);
    mdelay(1);     
    gpio_direction_output(CAM_MTP9111_RST_GPIO, 0);
    mdelay(1);     
    
    antares_mt9p111_torch_off() ;
	return 0;
}

struct mt9p111_platform_data antares_mt9p111_data = {
	.power_on = antares_mt9p111_power_on,
	.power_off = antares_mt9p111_power_off,
	.torch_on = antares_mt9p111_torch_on,
	.torch_off = antares_mt9p111_torch_off,
};
#endif

#ifdef CONFIG_VIDEO_MT9D115_SCORPIO
static const struct camera_gpios mt9d115_gpio_keys[] = {
	[0] = CAMERA_GPIO("en_avdd_csi", AVDD_DSI_CSI_ENB_GPIO, 1, 1),
};

static int antares_mt9d115_power_on(void)
{
	int ret = 0;


    gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 1);
    mdelay(1);
	gpio_direction_output(CAM_MTP9115_PWD_GPIO, 0);
    mdelay(1);        
    gpio_direction_output(CAM_MTP9115_RST_GPIO, 1);
    mdelay(1);     
 
	antares_mt9p111_torch_off() ;
	return ret;
}

static int antares_mt9d115_power_off(void)
{

    gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);
    mdelay(1);
	gpio_direction_output(CAM_MTP9115_PWD_GPIO, 1);
    mdelay(1);     
    gpio_direction_output(CAM_MTP9115_RST_GPIO, 0);
    mdelay(1);     
    

	return 0;
}

struct mt9d115_platform_data antares_mt9d115_data = {
	.power_on = antares_mt9d115_power_on,
	.power_off = antares_mt9d115_power_off,
};
#endif

int __init antares_camera_init(void)
{
	int ret = 0;
	int i=0;


	//Flash light

    tegra_gpio_enable(TEGRA_GPIO_PBB5);
	gpio_request(TEGRA_GPIO_PBB5, "cam_flash_en");
	gpio_direction_output(TEGRA_GPIO_PBB5, 0);

    tegra_gpio_enable(AVDD_DSI_CSI_ENB_GPIO);
    gpio_request(AVDD_DSI_CSI_ENB_GPIO, "cam_csi_port");
    gpio_direction_output(AVDD_DSI_CSI_ENB_GPIO, 0);

#ifdef CONFIG_VIDEO_MT9P111_SCORPIO

	tegra_gpio_enable(CAM_MTP9111_PWD_GPIO);
	gpio_request(CAM_MTP9111_PWD_GPIO, "cam_5m_pwd");
	gpio_direction_output(CAM_MTP9111_PWD_GPIO, 1);    
    tegra_gpio_enable(CAM_MTP9111_RST_GPIO);
    gpio_request(CAM_MTP9111_RST_GPIO, "cam_5m_rst");
	gpio_direction_output(CAM_MTP9111_RST_GPIO, 0);


#endif

#ifdef CONFIG_VIDEO_MT9D115_SCORPIO

	tegra_gpio_enable(CAM_MTP9115_PWD_GPIO);
	gpio_request(CAM_MTP9115_PWD_GPIO, "cam_2m_pwd");
	gpio_direction_output(CAM_MTP9115_PWD_GPIO, 1);
    
    tegra_gpio_enable(CAM_MTP9115_RST_GPIO);
    gpio_request(CAM_MTP9115_RST_GPIO, "cam_2m_rst");
    
	gpio_direction_output(CAM_MTP9115_RST_GPIO, 0);


#endif


	return 0;
}


static struct nvc_torch_pin_state scorpio_ssl3250a_pinstate = {
	.mask		= 0x0040, /* VGP6 */
	.values		= 0x0040,
};
static struct ssl3250a_platform_data scorpio_ssl3250a_pdata = {
        .dev_name       = "torch",
        .pinstate       = &scorpio_ssl3250a_pinstate,
        .gpio_act       = CAMERA_FLASH_ACT_GPIO,
};

static void scorpio_isl29018_init(void)
{
	tegra_gpio_enable(ISL29018_IRQ_GPIO);
	gpio_request(ISL29018_IRQ_GPIO, "isl29018");
	gpio_direction_input(ISL29018_IRQ_GPIO);
}

#ifdef CONFIG_SENSORS_AK8975
static void scorpio_akm8975_init(void)
{
	tegra_gpio_enable(AKM8975_IRQ_GPIO);
	gpio_request(AKM8975_IRQ_GPIO, "akm8975");
	gpio_direction_input(AKM8975_IRQ_GPIO);
}
#endif

static void scorpio_nct1008_init(void)
{
	tegra_gpio_enable(NCT1008_THERM2_GPIO);
	gpio_request(NCT1008_THERM2_GPIO, "temp_alert");
	gpio_direction_input(NCT1008_THERM2_GPIO);
}

static struct nct1008_platform_data scorpio_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = false,
	.conv_rate = 0x08,
	.offset = 0,
	.hysteresis = 0,
	.shutdown_ext_limit = 115,
	.shutdown_local_limit = 120,
	.throttling_ext_limit = 90,
	.alarm_fn = tegra_throttling_enable,
};

static const struct i2c_board_info scorpio_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ2),
	},
};

static const struct i2c_board_info scorpio_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO("bq20z75", 0x0B),
	},
};

static const struct i2c_board_info scorpio_i2c3_board_info_ssl3250a[] = {
	{
		I2C_BOARD_INFO("ssl3250a", 0x30),
		.platform_data = &scorpio_ssl3250a_pdata,
	},
};

static struct i2c_board_info scorpio_i2c4_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.irq = TEGRA_GPIO_TO_IRQ(NCT1008_THERM2_GPIO),
		.platform_data = &scorpio_nct1008_pdata,
	},

#ifdef CONFIG_SENSORS_AK8975
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq = TEGRA_GPIO_TO_IRQ(AKM8975_IRQ_GPIO),
	},
#endif
};

#ifdef CONFIG_VIDEO_MT9P111_SCORPIO
static struct i2c_board_info antares_i2c6_board_info[] = {
	{
		I2C_BOARD_INFO("mt9p111", 0x3D),
		.platform_data = &antares_mt9p111_data,
	},
};
#endif

#ifdef CONFIG_VIDEO_MT9D115_SCORPIO
static struct i2c_board_info antares_i2c7_board_info[] = {
	{
		I2C_BOARD_INFO("mt9d115", 0x3C),
		.platform_data = &antares_mt9d115_data,
	},
};
#endif

#ifdef CONFIG_MPU_SENSORS_MPU3050
#define SENSOR_MPU_NAME "mpu3050"
static struct mpu3050_platform_data mpu3050_data = {
	.int_config  = 0x10,
	#ifdef SENSOR_ER_VERSION
	.orientation = {  1,  0,  0,
                      0, -1,  0,
                      0,  0, -1 },	/* Orientation matrix for MPU on scorpio */
	#else
	.orientation = {  0, 1,  0,
					  1,  0,  0,
					  0,  0, -1 },	/* Orientation matrix for MPU on scorpio */
	#endif
	.level_shifter = 0,
	.accel = {
#ifdef CONFIG_MPU_SENSORS_KXTF9
	.get_slave_descr = get_accel_slave_descr,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN4),
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 0,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x0F,
	#ifdef SENSOR_ER_VERSION
	.orientation = {  0, -1,  0,
					 -1,  0,  0,
					  0,  0, -1 },	/* Orientation matrix for Accel on scorpio */
	},
	#else
	.orientation = { -1,  0,  0,
					  0,  1,  0,
					  0,  0, -1 },  /* Orientation matrix for Accel on scorpio */
	},
	#endif

	.compass = {
#ifdef CONFIG_MPU_SENSORS_AK8975
	.get_slave_descr = get_compass_slave_descr,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PN5),
#else
	.get_slave_descr = NULL,
#endif
	.adapt_num   = 4,            /* bus number 4 on scorpio */
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x0C,
	.orientation = {  1,  0,  0,
			  0,  1,  0,
			  0,  0,  1 },  /* Orientation matrix for AKM on scorpio */
	},
};

static struct i2c_board_info __initdata mpu3050_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PZ4),
		.platform_data = &mpu3050_data,
	},
};

static void scorpio_mpuirq_init(void)
{
	pr_info("*** MPU START *** scorpio_mpuirq_init...\n");
	tegra_gpio_enable(TEGRA_GPIO_PZ4);
	gpio_request(TEGRA_GPIO_PZ4, SENSOR_MPU_NAME);
	gpio_direction_input(TEGRA_GPIO_PZ4);
	tegra_gpio_enable(TEGRA_GPIO_PN4);
	gpio_request(TEGRA_GPIO_PN4, "KXTF9");
	gpio_direction_input(TEGRA_GPIO_PN4);
	pr_info("*** MPU END *** scorpio_mpuirq_init...\n");
}
#endif

int __init scorpio_sensors_init(void)
{
	struct board_info BoardInfo;

	scorpio_isl29018_init();
#ifdef CONFIG_SENSORS_AK8975
	scorpio_akm8975_init();
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
	scorpio_mpuirq_init();
#endif
	scorpio_nct1008_init();

	i2c_register_board_info(0, scorpio_i2c0_board_info,
		ARRAY_SIZE(scorpio_i2c0_board_info));

	tegra_get_board_info(&BoardInfo);

	/*
	 * battery driver is supported on FAB.D boards and above only,
	 * since they have the necessary hardware rework
	 */
	if (BoardInfo.sku > 0) {
		i2c_register_board_info(2, scorpio_i2c2_board_info,
			ARRAY_SIZE(scorpio_i2c2_board_info));
	}

	i2c_register_board_info(3, scorpio_i2c3_board_info_ssl3250a,
		ARRAY_SIZE(scorpio_i2c3_board_info_ssl3250a));

	i2c_register_board_info(4, scorpio_i2c4_board_info,
		ARRAY_SIZE(scorpio_i2c4_board_info));

#ifdef CONFIG_VIDEO_MT9P111_SCORPIO
	i2c_register_board_info(3, antares_i2c6_board_info,
		ARRAY_SIZE(antares_i2c6_board_info));
#endif

#ifdef CONFIG_VIDEO_MT9D115_SCORPIO
	i2c_register_board_info(3, antares_i2c7_board_info,
		ARRAY_SIZE(antares_i2c7_board_info));
#endif

#ifdef CONFIG_MPU_SENSORS_MPU3050
	i2c_register_board_info(0, mpu3050_i2c0_boardinfo,
		ARRAY_SIZE(mpu3050_i2c0_boardinfo));
#endif

	return 0;
}

late_initcall(antares_camera_init);
