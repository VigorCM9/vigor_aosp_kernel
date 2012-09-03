/* linux/arch/arm/mach-msm/display/vigor-panel.c
 *
 * Copyright (c) 2011 HTC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/io.h>
#include <asm/mach-types.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#include <mach/panel_id.h>
#include <mach/msm_bus_board.h>
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <mach/debug_display.h>

#include "../devices.h"
#include "../board-vigor.h"
#include "../devices-msm8x60.h"
#include "../../../../drivers/video/msm_8x60/mdp_hw.h"
#if defined (CONFIG_FB_MSM_MDP_ABL)
#include <linux/fb.h>
#endif

/*
 * =============== Display related function (BEGIN) ===============
 */
#define	VIGOR_DC_DC	(1<<0)
#define GPIO_LCM_RST_N	137
#define PM8058_GPIO_BASE			NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)

static struct regulator *v_lcm;
static struct regulator *v_lcmio;
unsigned int vigor_get_engineerid(void);

static void vigor_panel_power(int on)
{
	static int init;
	int ret;
	int rc;

	char *lcm_str = "8058_l12";
	char *lcmio_str = "8901_lvs1";

	struct pm_gpio gpio_cfg_LCM_3V = {
		.direction	= PM_GPIO_DIR_OUT,
		.output_value	= 0,
		.output_buffer	= PM_GPIO_OUT_BUF_CMOS,
		.pull		= PM_GPIO_PULL_NO,
		.out_strength	= PM_GPIO_STRENGTH_HIGH,
		.function	= PM_GPIO_FUNC_NORMAL,
		.vin_sel	= PM8058_GPIO_VIN_S3,	/* S3 1.8 V */
		.inv_int_pol	= 0,
	};

	/* If panel is already on (or off), do nothing. */
	if (!init) {
		if (vigor_get_engineerid() < VIGOR_DC_DC) {
			v_lcm = regulator_get(NULL, lcm_str);
			if (IS_ERR(v_lcm)) {
				pr_err("%s: unable to get %s\n", __func__, lcm_str);
				goto fail;
			}
		} else
			pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(VIGOR_LCM_3V), &gpio_cfg_LCM_3V);

		v_lcmio = regulator_get(NULL, lcmio_str);
		if (IS_ERR(v_lcmio)) {
			pr_err("%s#%d: unable to get %s\n", __func__, __LINE__, lcmio_str);
			goto fail;
		}

		if (vigor_get_engineerid() < VIGOR_DC_DC) {
			ret = regulator_set_voltage(v_lcm, 3200000, 3200000);
			if (ret) {
				pr_err("%s: error setting %s voltage\n", __func__, lcm_str);
				goto fail;
			}
		}

		/* LCM Reset */
		rc = gpio_request(GPIO_LCM_RST_N, "LCM_RST_N");
		if (rc) {
			printk(KERN_ERR "%s:LCM gpio %d request"
				"failed\n", __func__,  GPIO_LCM_RST_N);
			return;
		}

		init = 1;
	}

	if (vigor_get_engineerid() < VIGOR_DC_DC) {
		if (!v_lcm || IS_ERR(v_lcm)) {
			pr_err("%s: %s is not initialized\n", __func__, lcm_str);
			return;
		}
	}

	if (!v_lcmio || IS_ERR(v_lcmio)) {
		pr_err("%s: %s is not initialized\n", __func__, lcmio_str);
		return;
	}

	if (on) {
		if (vigor_get_engineerid() < VIGOR_DC_DC) {
			if (regulator_enable(v_lcm)) {
				pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcm_str);
				return;
			}
		} else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_LCM_3V), 1);

		mdelay(5);

		if (regulator_enable(v_lcmio)) {
			pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcmio_str);
			return;
		}

		mdelay(10);
		gpio_set_value(GPIO_LCM_RST_N, 1);
		mdelay(1);
		gpio_set_value(GPIO_LCM_RST_N, 0);
		mdelay(1);
		gpio_set_value(GPIO_LCM_RST_N, 1);
		mdelay(20);
	} else {
		mdelay(110);
		gpio_set_value(GPIO_LCM_RST_N, 0);
		mdelay(5);
		if (vigor_get_engineerid() < VIGOR_DC_DC) {
			if (regulator_disable(v_lcm)) {
				pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcm_str);
				return;
			}
		} else
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(VIGOR_LCM_3V), 0);
		mdelay(5);
		if (regulator_disable(v_lcmio)) {
			pr_err("%s: Unable to enable the regulator: %s\n", __func__, lcmio_str);
			return;
		}
	}
	return;

fail:
	if (v_lcm)
		regulator_put(v_lcm);
	if (v_lcmio)
		regulator_put(v_lcmio);
}

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_smi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 147460000,
		.ib = 184325000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_sd_ebi_vectors[] = {
	/* Default case static display/UI/2d/3d if FB SMI */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 280780800,
		.ib = 701952000,
	},
};

static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 35097600,
		.ib = 87744000,
	},
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 315878400,
		.ib = 789696000,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 105292800,
		.ib = 263232000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 386073600,
		.ib = 965184000,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 236908800,
		.ib = 592272000,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 517689600,
		.ib = 1294224000,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_smi_vectors),
		mdp_sd_smi_vectors,
	},
	{
		ARRAY_SIZE(mdp_sd_ebi_vectors),
		mdp_sd_ebi_vectors,
	},

	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	/* For now, 0th array entry is reserved.
	 * Please leave 0 as is and don't use it
	 */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 236908800,
		.ib = 592272000,

	},
	/* Master and slaves can be from different fabrics */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 517689600,
		.ib = 1294224000,
	},
};

static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};

static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
};
#endif	/* CONFIG_MSM_BUS_SCALING */

static uint32_t lcd_on_gpio[] = {
	GPIO_CFG(VIGOR_GPIO_LCM_ID0,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_GPIO_LCM_ID1,  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static uint32_t lcd_off_gpio[] = {
	GPIO_CFG(VIGOR_GPIO_LCM_ID0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(VIGOR_GPIO_LCM_ID1,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err(" %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int mipi_panel_power(int on)
{
	int flag_on = !!on;
	static int mipi_power_save_on;

	if (mipi_power_save_on == flag_on)
		return 0;

	mipi_power_save_on = flag_on;

	/* config panel id GPIO per H/W spec */
	if (on)
		config_gpio_table(lcd_on_gpio, ARRAY_SIZE(lcd_on_gpio));
	else
		config_gpio_table(lcd_off_gpio, ARRAY_SIZE(lcd_off_gpio));

	vigor_panel_power(on);

	return 0;
}

static struct mipi_dsi_platform_data mipi_pdata = {
	.vsync_gpio = 28,
	.dsi_power_save   = mipi_panel_power,
};

int mdp_core_clk_rate_table[] = {
	128000000,
	128000000,
	200000000,
	200000000,
};

#if defined (CONFIG_FB_MSM_MDP_ABL)
static struct gamma_curvy gamma_tbl = {
	.gamma_len = 33,
	.bl_len = 8,
	.ref_y_gamma = {0, 1, 1, 2, 4, 7, 11, 17, 24, 35, 44, 55,
                72, 87, 109, 130, 147, 171, 205, 242, 270,
                308, 351, 392, 436, 504, 554, 616, 686, 793,
                867, 946, 1024},
	.ref_y_shade = {0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352,
                384, 416, 448, 480, 512, 544, 576, 608, 640, 672, 704,
                736, 768, 800, 832, 864, 896, 928, 960, 992, 1024},
	.ref_bl_lvl = {0, 44, 116, 185, 297, 466, 751, 1024},
	.ref_y_lvl = {0, 138, 218, 298, 424, 601, 818, 1024},
};
#endif

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 28,
	.mdp_core_clk_rate = 128000000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
#if defined (CONFIG_FB_MSM_MDP_ABL)
	.abl_gamma_tbl = &gamma_tbl,
#endif

};

/* parameters for backlight value mapping */
#define BACKLIGHT_MAX 255

#define ORIG_PWM_MAX 255
#define ORIG_PWM_DEF 143
#define ORIG_PWM_MIN 30

#define MAP_PWM_MAX	255
#define MAP_PWM_DEF 	110
#define MAP_PWM_MIN	7

static unsigned char vigor_shrink_pwm(int val)
{
	unsigned char shrink_br;

	/* define line segments for Vigor */
	if (val <= ORIG_PWM_MIN)
		shrink_br = MAP_PWM_MIN;
	else if (val > ORIG_PWM_MIN && val <= ORIG_PWM_DEF)
		shrink_br = MAP_PWM_MIN +
			(val-ORIG_PWM_MIN)*(MAP_PWM_DEF-MAP_PWM_MIN)/(ORIG_PWM_DEF-ORIG_PWM_MIN);
	else
		shrink_br = MAP_PWM_DEF +
			(val-ORIG_PWM_DEF)*(MAP_PWM_MAX-MAP_PWM_DEF)/(ORIG_PWM_MAX-ORIG_PWM_DEF);

	pr_info("brightness orig = %d, transformed=%d\n", val, shrink_br);

	return shrink_br;
}

static struct msm_panel_common_pdata mipi_panel_data = {
	.shrink_pwm = vigor_shrink_pwm,
};

static struct platform_device mipi_dsi_cmd_panel_device = {
	.name = "mipi_himax",
	.id = 0,
	.dev = {
		.platform_data = &mipi_panel_data,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (panel_type == PANEL_ID_VIG_SHARP_HX ||
		panel_type == PANEL_ID_VIG_SHARP_HX_C2 ||
		panel_type == PANEL_ID_VIG_SHARP_HX_C25 ||
		panel_type == PANEL_ID_VIG_SHARP_HX_C3 ||
		panel_type == PANEL_ID_VIG_CHIMEI_HX_C3 ||
		panel_type == PANEL_ID_VIG_CHIMEI_HX_C25 ||
		panel_type == PANEL_ID_VIG_CHIMEI_HX) {
		if (!strcmp(name, "mipi_video_himax_720p"))
			return 0;
	}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
	if (!strcmp(name, "hdmi_msm"))
		return 0;
#endif

	pr_warning("%s: not supported '%s' (0x%x)", __func__, name, panel_type);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.width = 53,
	.height = 95,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id	 = 0,
	.dev.platform_data = &msm_fb_pdata,
};

static void __init msm_fb_add_devices(void)
{
	printk(KERN_INFO "panel ID= 0x%x\n", panel_type);
	msm_fb_register_device("mdp", &mdp_pdata);

	if (panel_type != PANEL_ID_NONE)
		msm_fb_register_device("mipi_dsi", &mipi_pdata);

#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

int __init vigor_init_panel(struct resource *res, size_t size)
{
	int ret;

	PR_DISP_INFO("%s: res=%p, size=%d\n", __func__, res, size);

	msm_fb_device.resource = res;
	msm_fb_device.num_resources = size;

	ret = platform_device_register(&msm_fb_device);
	ret = platform_device_register(&mipi_dsi_cmd_panel_device);

	msm_fb_add_devices();

	return 0;
}
/*
 * =============== Display related function (END) ===============
 */

