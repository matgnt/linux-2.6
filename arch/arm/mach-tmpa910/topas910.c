/*
 * arch/arm/mach-tmpa910/topas910.c -- Topas 910 machine
 *
 *	Copyright (C) 2008 bplan GmbH
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#include <asm/system.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <video/tmpa910_fb.h>
#include <asm/arch/gpio.h>

#include <asm/mach/arch.h>
#include <asm/arch/hardware.h>
#include <asm/arch/ts.h>
#include <asm/arch/tmpa910_regs.h>
#include <asm/serial.h>
#include <asm/dma.h>

#if defined(CONFIG_USB_ISP1362_HCD)
#include <linux/usb/isp1362.h>
#endif

#include "topas910.h"

/*********/
/*********/
//#define __DEBUG__
#include <linux/debug.h>

/*********/
/*********/
int topas910_io_mapped = 0;

static struct map_desc tmpa910_io_desc[] __initdata = {
	{
		.virtual = TMPA910_IO_VIRT_BASE,
		.pfn		 = __phys_to_pfn(TMPA910_IO_PHYS_BASE),
		.length	 = TMPA910_IO_SIZE,
		.type		 = MT_DEVICE,
	}
};


void __init topas910_map_io(void)
{
	NPRINTK("->");
	iotable_init(tmpa910_io_desc, ARRAY_SIZE(tmpa910_io_desc));
	topas910_io_mapped = 1;
}


/*********/
/*********/
static void dummy_release(struct device *dev)
{
        /* normally not freed */
}

static u64  topas910_dmamask = 0xffffffffUL;
/*********/
/*********/
#ifdef CONFIG_USB_ISP1362_HCD
static struct isp1362_platform_data isp1362_priv = {
        .sel15Kres = 1,
        .is_otg = 1,
        .clknotstop = 0,
        .oc_enable =1,
        .int_act_high = 0,
        .int_edge_triggered = 0,
        .remote_wakeup_connected = 0,
        .no_power_switching = 0,
        .power_switching_mode = 1,
        .reset = NULL,
        .clock = NULL,
};

static struct resource tmpa910_resource_isp1362[] = {
        [0] = {
                .start  = 0xa0000008,
                .end    = 0xa0000009,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = 0xa0000000,
                .end    = 0xa0000001,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = INTR_VECT_GPIOP,
                .end    = INTR_VECT_GPIOP,
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};

static struct platform_device topas910_isp1362_device = {
        .name           = "isp1362-hcd",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(tmpa910_resource_isp1362),
        .resource       = tmpa910_resource_isp1362,
        .dev = {
					.platform_data = &isp1362_priv,
						.release        = dummy_release, // not needed
					.coherent_dma_mask = 0xffffffff,
        },
};
#endif


static struct resource dm9000_resources[] = {
        [0] = {
                .start  = 0x60000002,
                .end    = 0x60000003,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = 0x60001002,
                .end    = 0x60001003,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = TOPAS910_INT_DM9000,
                .end    = TOPAS910_INT_DM9000,
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
        },
};


static struct platform_device topas910_dm9000_device = {
        .name           = "dm9000",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(dm9000_resources),
        .resource       = dm9000_resources,
        .dev = {
		.release        = dummy_release, // not needed
		.coherent_dma_mask = 0xffffffff,
        },
};
/*********/
/*********/
static struct resource tmpa910_resource_uart0[] = {
	{
		.start	= 0xf2000000,
		.end	= 0xf2000000+0x100,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= 10,
		.end	= 10,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};


struct platform_device tmpa910_device_uart0= {
	.name		= "tmpa910-uart",
	.id		= 0,
	.resource	= tmpa910_resource_uart0,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_uart0),
};

/*********/
/*********/
/*
 * Tousch screen
*/
static struct resource tmpa910_resource_ts[] = {
	{
		.start	= TS_BASE,
		.end	= TS_BASE+0x40,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= ADC_BASE,
		.end	= ADC_BASE+0x100,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_GPIOD,
		.end	= INTR_VECT_GPIOD,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}, {
		.start	= INTR_VECT_ADC,
		.end	= INTR_VECT_ADC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};

struct platform_device tmpa910_device_ts = {
	.name		= "tmpa910_ts",
	.id		= 0,
	.dev =
	{
		.platform_data = NULL
	},
	.resource	= tmpa910_resource_ts,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_ts),
};


/*********/
/*********/
static struct resource tmpa910_resource_lcdc[] = {
	{
		.start	= LCDC_BASE,
		.end	= LCDC_BASE+0x400,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= FB_OFFSET,
		.end	= FB_OFFSET+FB_SIZE,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= INTR_VECT_LCDC,
		.end	= INTR_VECT_LCDC,
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	}
};


static struct tmpa910_lcdc_platforminfo topas910_v1_lcdc_platforminfo;


struct platform_device tmpa910_device_lcdc= {
	.name		= "tmpa910_lcdc",
	.id		= 0,
	.resource	= tmpa910_resource_lcdc,
	.num_resources	= ARRAY_SIZE(tmpa910_resource_lcdc),
};

static struct platform_device *devices[] __initdata = {
	&topas910_dm9000_device,
	&tmpa910_device_uart0,
	&tmpa910_device_ts,
	&tmpa910_device_lcdc,
#ifdef CONFIG_USB_ISP1362_HCD
	&topas910_isp1362_device
#endif
};

static void __init
_setup_lcdc_device(void)
{
	uint32_t *LCDReg;
	int width  = 320;
	int height = 240;

	topas910_v1_lcdc_platforminfo.width  = width;
	topas910_v1_lcdc_platforminfo.height = height;
	topas910_v1_lcdc_platforminfo.depth  = 32;
	topas910_v1_lcdc_platforminfo.pitch  = width*4;

	LCDReg = topas910_v1_lcdc_platforminfo.LCDReg;
	LCDReg[0] =
				  ( ((height/16)-1) << 2)	// pixel per line
				| ( (8-1) << 8 ) 				// tHSW. Horizontal sync pulse
				| ( (8-1) << 16 ) 			// tHFP, Horizontal front porch
				| ( (8-1) << 24 ) 			// tHBP, Horizontal back porch
				;

	LCDReg[1] =
				(2 << 24) 		// tVBP
				| (2 << 16) 		// tVFP
				| ((2-1) << 10) 		// tVSP
				| (width-1);

	LCDReg[2] = ((width-1)<<16) | 0x0000e | 1<<13 | 0<<12 | 0<<11;
	LCDReg[3] = 0;
	LCDReg[4]	= (0x5<<1)  | (1<<5) | (1<<11);
	tmpa910_device_lcdc.dev.platform_data = &topas910_v1_lcdc_platforminfo;
}

/***/
static void __init
topas910_init(void)
{

	NPRINTK("->");

	platform_bus.coherent_dma_mask=0xffffffff;
	platform_bus.dma_mask=&topas910_dmamask;

	_setup_lcdc_device();

  platform_add_devices(devices, ARRAY_SIZE(devices));


}


MACHINE_START(TOPAS910, "Toshiba Topas910")
        /* Maintainer:  */
        .phys_io        = TMPA910_IO_PHYS_BASE,
				.boot_params    = 0,
        .io_pg_offst    = (io_p2v(TMPA910_IO_PHYS_BASE) >> 18) & 0xfffc,
        .map_io         = topas910_map_io,
        .init_irq       = topas910_init_irq,
        .timer          = &topas910_timer,
        .init_machine   = topas910_init,
MACHINE_END


void tmpa910_led_blink(void)
{
	if(topas910_io_mapped)
	{
		uint32_t reg;

		reg = _in32(0xf08013FC);
		reg ++;
		_out32(0xf08013FC, reg);
	}
}

EXPORT_SYMBOL(tmpa910_led_blink);
