/*
 * arch/arm/mach-tegra/board-antares.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/gpio_keys.h>
#include <linux/gpio_switch.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/mfd/tps6586x.h>
#include <linux/memblock.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/tegra_uart.h>
#include <linux/antares_dock.h>
#include <linux/interrupt.h>

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
#include <linux/i2c/panjit_ts.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
#include <linux/i2c/atmel_maxtouch.h>
#endif

#ifdef CONFIG_SND_SOC_FM34
#include <sound/fm34.h>
#endif

#include <sound/wm8903.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_wm8903_pdata.h>
#include <asm/setup.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>

#include "board.h"
#include "clock.h"
#include "board-antares.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t2.h"
#include "pm.h"

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

#ifdef CONFIG_TEGRA_ODM_DMIEEP
struct tag_platform_data {
    int             num_data;
    unsigned char*  data;
};

static bool g_tag_data_ready = false;
static unsigned char calibrate_data[60] = {0};

static struct tag_platform_data antares_tag_platform_data = {
	.data		= calibrate_data,
	.num_data	= ARRAY_SIZE(calibrate_data),
};

static struct platform_device tegra_dmieep_device = {
    .name = "dmieep",
    .id = -1,
    .dev = {
		.platform_data  = &antares_tag_platform_data,
    },
};
#endif

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "cdev2",
};

static struct resource antares_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device antares_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(antares_bcm4329_rfkill_resources),
	.resource       = antares_bcm4329_rfkill_resources,
};

static void __init antares_bt_rfkill(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", antares_bcm4329_rfkill_device.name, \
				"blink", NULL);
	return;
}

static struct resource antares_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device antares_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(antares_bluesleep_resources),
	.resource       = antares_bluesleep_resources,
};

static void __init antares_setup_bluesleep(void)
{
	platform_device_register(&antares_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);
	return;
}

static __initdata struct tegra_clk_init_table antares_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "blink",	"clk_32k",	32768,		false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	//{ "pwm",	"clk_32k",	32768,		false},
	{ "pwm",        "clk_m",        12000000,       false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_ulpi_config antares_ehci2_ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data antares_ehci2_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
	.phy_config = &antares_ehci2_ulpi_phy_config,
	.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
	.default_enable = true,
};

static struct tegra_i2c_platform_data antares_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data antares_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 100000, 10000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.slave_addr = 0x00FC,
	.scl_gpio		= {0, TEGRA_GPIO_PT5},
	.sda_gpio		= {0, TEGRA_GPIO_PT6},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data antares_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
	.scl_gpio		= {TEGRA_GPIO_PBB2, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB3, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data antares_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_dvc		= true,
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct wm8903_platform_data antares_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = ANTARES_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT) | WM8903_GP1_LVL | WM8903_GP1_DB,
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT) | WM8903_GP1_DIR | WM8903_GP1_IP_CFG,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
	.adc_digital_volume = 0xEF,
	.adc_analogue_volume = 0x1B,
	.dac_digital_volume = 0xBD,
	.dac_headphone_volume = 0x39,
	.dac_ext_hp_volume = 0x3A,
	.dac_speaker_volume = 0x3A,

};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &antares_wm8903_pdata,
	.irq = 0,
};

static void antares_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &antares_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &antares_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &antares_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &antares_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);

	i2c_register_board_info(0, &wm8903_board_info, 1);
}

#ifdef CONFIG_LEDS_GPIO
static struct gpio_led antares_leds[] = {
	{
		.name			= "RF_LED",
		.default_trigger	= "",
		.gpio			= TEGRA_GPIO_PD5,
		.active_low		= 0,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
		.retain_state_suspended = 1,
	},
};

static struct gpio_led_platform_data antares_leds_platform_data = {
	.leds		= antares_leds,
	.num_leds	= ARRAY_SIZE(antares_leds),
};

static struct platform_device antares_leds_device = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &antares_leds_platform_data,
	},
};

static void antares_leds_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(antares_leds); i++)
		tegra_gpio_enable(antares_leds[i].gpio);
}
#endif

static struct platform_device *antares_uart_devices[] __initdata = {
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data antares_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	antares_uart_devices[2] = &debug_uartd_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
	}
}

static void __init antares_uart_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	antares_uart_pdata.parent_clk_list = uart_parent_clk;
	antares_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uartb_device.dev.platform_data = &antares_uart_pdata;
	tegra_uartc_device.dev.platform_data = &antares_uart_pdata;
	tegra_uartd_device.dev.platform_data = &antares_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(antares_uart_devices,
				ARRAY_SIZE(antares_uart_devices));
}

#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button antares_keys[] = {
        [0] = GPIO_KEY(KEY_VOLUMEUP, PQ5, 0),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PQ4, 0),
	[2] = GPIO_KEY(KEY_POWER, PV2, 1),
	
};

#define PMC_WAKE_STATUS 0x14

static int antares_wakeup_key(void)
{
        int pending_wakeup_irq;
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	 writel(0xffffffff, IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
         pending_wakeup_irq = get_pending_wakeup_irq();

         if ((status & TEGRA_WAKE_GPIO_PV2) ||
            (pending_wakeup_irq == 362)) /* power button */
            return KEY_POWER;
         else if (status & TEGRA_WAKE_GPIO_PV3) /* AC adapter plug in/out */
            return KEY_POWER;
         else
            return KEY_RESERVED;
}

static struct gpio_keys_platform_data antares_keys_platform_data = {
	.buttons	= antares_keys,
	.nbuttons	= ARRAY_SIZE(antares_keys),
	.wakeup_key	= antares_wakeup_key,
};

static struct platform_device antares_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data	= &antares_keys_platform_data,
	},
};

static void antares_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(antares_keys); i++)
		tegra_gpio_enable(antares_keys[i].gpio);
}
#endif

#ifdef CONFIG_INPUT_GPIO_SWITCH
#define GPIO_SWITCH(_id, _gpio, _name)    \
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_name,      \
		.type = EV_SW,			\
		.debounce_interval = 10,	\
	}

static struct gpio_switch antares_switches[] = {
	[0] = GPIO_SWITCH(SW_ROTATE_LOCK, PH2, rotationlock),
};

static struct gpio_sw_platform_data antares_sw_platform_data = {
	.switches	= antares_switches,
	.nswitches	= ARRAY_SIZE(antares_switches),
};

static struct platform_device antares_switches_device = {
	.name	= "gpio-switches",
	.id	= 0,
	.dev	= {
		.platform_data	= &antares_sw_platform_data,
	},
};

static void antares_switches_init(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(antares_switches); i++)
		tegra_gpio_enable(antares_switches[i].gpio);
}
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct dock_platform_data dock_on_platform_data = {
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PG2),
		.gpio_num	= TEGRA_GPIO_PG2,
};

static struct platform_device tegra_dock_device =
{
    .name = "tegra_dock",
    .id   = -1,
    .dev = {
		.platform_data = &dock_on_platform_data,
	},
};
static struct tegra_wm8903_platform_data antares_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_ext_mic_en	= TEGRA_GPIO_AMIC_EN,
	.gpio_ext_mic_det	= TEGRA_GPIO_AMIC_DET,
	.gpio_hp_mute		= -1,
	.gpio_ext_hp_det	= TEGRA_GPIO_EXT_HP_DET,
	.gpio_int_spkr_en       = TEGRA_GPIO_INT_SPKR_EN,
};

static struct platform_device antares_audio_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data  = &antares_audio_pdata,
	},
};

static struct platform_device *antares_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_udc_device,
	&tegra_ehci2_device,
	&tegra_gart_device,
	&tegra_aes_device,
#ifdef CONFIG_LEDS_GPIO
	&antares_leds_device,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&antares_keys_device,
#endif
#ifdef CONFIG_INPUT_GPIO_SWITCH
	&antares_switches_device,
#endif
	&tegra_wdt_device,
	&tegra_avp_device,
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_spdif_device,
	&tegra_das_device,
	&tegra_pwfm1_device,
	&tegra_dock_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&antares_bcm4329_rfkill_device,
#ifdef CONFIG_TEGRA_ODM_DMIEEP
	&tegra_dmieep_device,
#endif
	&tegra_pcm_device,
	&antares_audio_device,
};


static struct mxt_platform_data atmel_mxt_info = {
	.x_line		= 27,
	.y_line		= 42,
	.x_size		= 768,
	.y_size		= 1366,
	.blen		= 0x20,
	.threshold	= 0x3C,
	.voltage	= 3300000,
	.orient		= MXT_ROTATED_90,
	.irqflags	= IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata i2c_info[] = {
	{
	 I2C_BOARD_INFO("atmel_mxt_ts", 0x5A),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &atmel_mxt_info,
	 },
};

static int __init antares_touch_init_atmel(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);
	tegra_gpio_enable(TEGRA_GPIO_PQ7);

	gpio_request(TEGRA_GPIO_PV6, "atmel-irq");
	gpio_direction_input(TEGRA_GPIO_PV6);

	gpio_request(TEGRA_GPIO_PQ7, "atmel-reset");
	gpio_direction_output(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, i2c_info, 1);

	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_PANJIT_I2C
static struct panjit_i2c_ts_platform_data panjit_data = {
	.gpio_reset = TEGRA_GPIO_PQ7,
};

static struct i2c_board_info __initdata antares_i2c_bus1_touch_info[] = {
	{
		I2C_BOARD_INFO("panjit_touch", 0x3),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
		.platform_data = &panjit_data,
	},
};

static int __init antares_touch_init_panjit(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);

	tegra_gpio_enable(TEGRA_GPIO_PQ7);
	i2c_register_board_info(0, antares_i2c_bus1_touch_info, 1);

	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MT_T9
/* Atmel MaxTouch touchscreen              Driver data */
/*-----------------------------------------------------*/
/*
 * Reads the CHANGELINE state; interrupt is valid if the changeline
 * is low.
 */
static u8 read_chg(void)
{
	return gpio_get_value(TEGRA_GPIO_PV6);
}

static u8 valid_interrupt(void)
{
	return !read_chg();
}

static struct mxt_platform_data Atmel_mxt_info = {
	/* Maximum number of simultaneous touches to report. */
	.numtouch = 10,
	// TODO: no need for any hw-specific things at init/exit?
	.init_platform_hw = NULL,
	.exit_platform_hw = NULL,
	.max_x = 1366,
	.max_y = 768,
	.valid_interrupt = &valid_interrupt,
	.read_chg = &read_chg,
};

static struct i2c_board_info __initdata i2c_info[] = {
	{
	 I2C_BOARD_INFO("maXTouch", MXT_I2C_ADDRESS),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	 .platform_data = &Atmel_mxt_info,
	 },
};

static int __init antares_touch_init_atmel(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);
	tegra_gpio_enable(TEGRA_GPIO_PQ7);

	gpio_set_value(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, i2c_info, 1);

	return 0;
}
#endif

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_irq = TPS6586X_INT_BASE + TPS6586X_INT_USB_DET,
			.vbus_gpio = TEGRA_GPIO_PD0,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = TEGRA_GPIO_PD3,
	},
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = true,
	},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
			.default_enable = true,
	},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.hotplug = 1,
			.default_enable = true,
	},
};

#ifdef CONFIG_TOUCHSCREEN_EGALAX_I2C
static const struct i2c_board_info antares_i2c_bus1_touch_info[] = {
	{
		I2C_BOARD_INFO("egalax_i2c", 0x4),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	},
};

static int __init antares_touch_init_egalax(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);

	i2c_register_board_info(0, antares_i2c_bus1_touch_info, 1);

	return 0;
}
#endif

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

static int __init antares_gps_init(void)
{
	struct clk *clk32 = clk_get_sys(NULL, "blink");
	if (!IS_ERR(clk32)) {
		clk_set_rate(clk32,clk32->parent->rate);
		clk_enable(clk32);
	}

	tegra_gpio_enable(TEGRA_GPIO_PZ3);
	return 0;
}

static void antares_power_off(void)
{
        int ret;
        int retry = 10;
        while(retry--){
                ret = tps6586x_power_off();
                if (ret)
                        pr_err("antares: failed to power off\n");
        }
        while(1);
}

static void __init antares_power_off_init(void)
{
	pm_power_off = antares_power_off;
}

static void antares_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));
	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	tegra_ehci3_device.dev.platform_data=&tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
}
#ifdef CONFIG_SND_SOC_FM34
const static u16 fm34_property[][2] = {
	{0x22C8, 0x0026},
	{0x22D2, 0x8A94},
	{0x22E3, 0x50C2},
	{0x22EE, 0x0000},
	{0x22F2, 0x0040},
	{0x22F6, 0x0002},
	{0x22F8, 0x8005},
	{0x22F9, 0x085F},
	{0x22FA, 0x2483},
	{0x2301, 0x0002},
	{0x2303, 0x0021},
	{0x2305, 0x0004},
	{0x2307, 0xF8F8},       // Entry MIC in Gain
	{0x2309, 0x0800},
	{0x230C, 0x1000},       // Leave MIC In Gain
	{0x230D, 0x0100},       // Speaker Out Gain
	{0x2310, 0x1880},
	{0x2325, 0x5000},
	{0x232F, 0x0080},
	{0x2332, 0x0200},
	{0x2333, 0x0020},
	{0x2339, 0x0010},
	{0x2357, 0x0100},
	{0x2391, 0x4000},
	{0x2392, 0x4000},
	{0x2393, 0x4000},
	{0x2394, 0x4000},
	{0x2395, 0x4000},
	{0x23CF, 0x0050},
	{0x23D5, 0x4B00},
	{0x23D7, 0x002A},
	{0x23E0, 0x4000},
	{0x23E1, 0x4000},
	{0x23E2, 0x4000},
	{0x23E3, 0x4000},
	{0x23E4, 0x4000},
	{0x3FD2, 0x0032},
	{0x22FB, 0x0000}
};

static struct fm34_conf fm34_conf = {
        .pwdn = TEGRA_GPIO_PU2,
        .rst = -1,
        .bp = -1,
        .cprop = sizeof(fm34_property)/sizeof(u16)/2,
        .pprop = (u16 *)fm34_property,
};

static const struct i2c_board_info antares_fm34_board_info[] = {
	{
		I2C_BOARD_INFO("fm34_i2c", 0x60),
		.platform_data = &fm34_conf,
	},
};

static int __init antares_fm34_init(void)
{
        tegra_gpio_enable(fm34_conf.pwdn);
        if(fm34_conf.bp != -1)
                tegra_gpio_enable(fm34_conf.bp);
        if(fm34_conf.rst != -1)
                tegra_gpio_enable(fm34_conf.rst);

        i2c_register_board_info(0, antares_fm34_board_info, 1);

        return 0;
}
#endif
static void __init tegra_antares_init(void)
{
#if defined(CONFIG_TOUCHSCREEN_PANJIT_I2C) || \
	defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)
	struct board_info BoardInfo;
#endif

	tegra_clk_init_from_table(antares_clk_init_table);
	antares_pinmux_init();
	antares_i2c_init();
	antares_uart_init();
	tegra_ehci2_device.dev.platform_data
		= &antares_ehci2_ulpi_platform_data;
	platform_add_devices(antares_devices, ARRAY_SIZE(antares_devices));
	tegra_ram_console_debug_init();

#ifdef CONFIG_SND_SOC_FM34
	antares_fm34_init();
#endif
	antares_sdhci_init();
	antares_charge_init();
	antares_regulator_init();
	antares_charger_init();
    antares_ec_init();

#if defined(CONFIG_TOUCHSCREEN_PANJIT_I2C) || \
	defined(CONFIG_TOUCHSCREEN_ATMEL_MT_T9)

	tegra_get_board_info(&BoardInfo);

	/* boards with sku > 0 have atmel touch panels */
	if (BoardInfo.sku) {
		pr_info("Initializing Atmel touch driver\n");
		antares_touch_init_atmel();
	} else {
		pr_info("Initializing Panjit touch driver\n");
		antares_touch_init_panjit();
	}
#elif defined(CONFIG_TOUCHSCREEN_EGALAX_I2C)
	pr_info("Initializing eGalax touch driver\n");
	antares_touch_init_egalax();
#endif

#ifdef CONFIG_LEDS_GPIO
	antares_leds_init();
#endif

#ifdef CONFIG_KEYBOARD_GPIO
	antares_keys_init();
#endif
#ifdef CONFIG_INPUT_GPIO_SWITCH
	antares_switches_init();
#endif
	antares_usb_init();
	antares_gps_init();
	antares_panel_init();
	antares_sensors_init();
	antares_bt_rfkill();
	antares_power_off_init();
	antares_emc_init();

	antares_setup_bluesleep();
	tegra_release_bootloader_fb();
}

int __init tegra_antares_protected_aperture_init(void)
{
	if (!machine_is_ventana())
		return 0;

	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_antares_protected_aperture_init);

void __init tegra_antares_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M + SZ_1M, SZ_16M);
	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(ANTARES, "antares")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_antares_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= tegra_antares_init,
MACHINE_END

/* NVidia bootloader tags */
#define ATAG_NVIDIA_TEGRA		0x41000801

static int __init parse_tegra_tag(const struct tag *tag)
{
	return 0;
}
__tagtable(ATAG_NVIDIA_TEGRA, parse_tegra_tag);

#ifdef CONFIG_TEGRA_ODM_DMIEEP
/* Antares bootloader tag */
#define ATAG_ANTARES            0x41000810

static int __init parse_tegra_antares_tag(const struct tag *tag)
{
    const char *addr = (const char *)&tag->hdr + sizeof(struct tag_header);
    int data_size;

    data_size = *(int *)addr;

    if(data_size == 60){
        memcpy(calibrate_data, addr, data_size);
        g_tag_data_ready = true;
	}
    else
        pr_info("The tag data size is wrong(%d)\n", data_size);

	return 0;
}
__tagtable(ATAG_ANTARES, parse_tegra_antares_tag);

void antares_guery_ram_normal_mode(bool *enable)
{
    char str[5];

    *enable = false;
    if(g_tag_data_ready)
    {
        // About the offset, please refer dmi_pri.h to find the detail.
        //printk(KERN_INFO "*** [antares_guery_ram_normal_mode] data=%x%x%x%x\n", calibrate_data[24], calibrate_data[25], calibrate_data[26], calibrate_data[27]);
        memset(str, 0x00, 5);
        memcpy(str, &calibrate_data[24], 4);
        if(str[0]=='b' && str[1]=='e' && str[2]=='e' && str[3]=='f')
            *enable = true;
        else
          printk("Unknown Hynix memory tag: %c%c%c%c\n", str[0], str[1], str[2], str[3]);
    }
}
EXPORT_SYMBOL(antares_guery_ram_normal_mode);
#endif
