/* linux/arch/arm/mach-omap2/board-justin-wifi.c
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>

#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/wifi_tiwlan.h>

#include "mux.h"
#include "board-justin-wifi.h"

static int justin_wifi_cd;		/* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

/* LGE_CHANGE_S, [younggil.lee@lge.com], 2011-05-04, <add Setting enable Wifi Host wakeup> */
void config_wlan_mux(void)
{
	omap_mux_init_gpio(JUSTIN_WIFI_IRQ_GPIO, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(JUSTIN_WIFI_PMENA_GPIO, OMAP_PIN_OUTPUT);
}
/* LGE_CHANGE_E, [younggil.lee@lge.com], 2011-05-04, <add Setting enable Wifi Host wakeup> */

int omap_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

int omap_wifi_status(int irq)
{
	return justin_wifi_cd;
}


int justin_wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	justin_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_info("%s: Nobody to notify\n", __func__);
	return 0;
}

#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(justin_wifi_set_carddetect);
#endif

static int justin_wifi_power_state;

int justin_wifi_power(int on)
{
#if 0

//lewislee
//#define CONTROL_PADCONF_ETK_D9			0x480025EC   /* WLAN_EN */
//#define CONTROL_PADCONF_ETK_D10			0x480025F0   /* WLAN_IRQ */
//#define CONTROL_PADCONF_GPIO_OE			0x48310034   /* WLAN_EN GPIO OE */

//#define CONTROL_PADCONF_MMC3_CLK	   	0x480025D8  /* mmc3_cmd */
//#define CONTROL_PADCONF_MMC3_CMD	   	0x480025D8  /* mmc3_cmd */

	omap_writel(0x480025EC, 0x361c0000);
#endif

	pr_info("%s: %d\n", __func__, on);
	gpio_set_value(JUSTIN_WIFI_PMENA_GPIO, on);
	justin_wifi_power_state = on;

	//bill.jung@lge.com - Add delay
	msleep(300);
	
	return 0;
}

#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(justin_wifi_power);
#endif

static int justin_wifi_reset_state;
int justin_wifi_reset(int on)
{
	pr_info("%s: %d\n", __func__, on);
	justin_wifi_reset_state = on;
	return 0;
}

#ifndef CONFIG_WIFI_CONTROL_FUNC
EXPORT_SYMBOL(justin_wifi_reset);
#endif

struct wifi_platform_data justin_wifi_control = {
	.set_power	= justin_wifi_power,
	.set_reset	= justin_wifi_reset,
	.set_carddetect	= justin_wifi_set_carddetect,
};

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct resource justin_wifi_resources[] = {
	[0] = {
		.name		= "device_wifi_irq",
		.start		= OMAP_GPIO_IRQ(JUSTIN_WIFI_IRQ_GPIO),
		.end		= OMAP_GPIO_IRQ(JUSTIN_WIFI_IRQ_GPIO),
		.flags		= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device justin_wifi_device = {
	.name		= "device_wifi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(justin_wifi_resources),
	.resource	= justin_wifi_resources,
	.dev		= {
		.platform_data = &justin_wifi_control,
	},
};
#endif

static int __init justin_wifi_init(void)
{
	int ret;

	pr_info("%s: start\n", __func__);

	ret = gpio_request(JUSTIN_WIFI_IRQ_GPIO, "wifi_irq");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			JUSTIN_WIFI_IRQ_GPIO);
		goto out;
	}

	ret = gpio_request(JUSTIN_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		pr_err("%s: can't reserve GPIO: %d\n", __func__,
			JUSTIN_WIFI_PMENA_GPIO);
		gpio_free(JUSTIN_WIFI_IRQ_GPIO);
		goto out;
	}

	gpio_direction_input(JUSTIN_WIFI_IRQ_GPIO);	
	gpio_direction_output(JUSTIN_WIFI_PMENA_GPIO, 0);

#ifdef CONFIG_WIFI_CONTROL_FUNC
	ret = platform_device_register(&justin_wifi_device);
#endif
out:
	return ret;
}

device_initcall(justin_wifi_init);
