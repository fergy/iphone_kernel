#include <asm/mach/arch.h>
#include <mach/cpu.h>
#include <mach/time.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <plat/cpu.h>

static struct platform_device fbdev = {
	.name = "s5l8920_fb",

	.dev = {
		.platform_data = __phys_to_virt(0xF700000),
	},
};

static void __init atv2g_init(void)
{
	s5l8930_init();

	//platform_device_register(&fbdev);

	// TODO: Add atv2g devices here!
}

MACHINE_START(APPLETV_2G, "AppleTV 2G")
	/* Maintainer: iDroid Project */
	.map_io		= s5l8930_map_io,
	.init_irq	= s5l8930_init_irq,
	.timer		= &s5l8930_timer,
	.init_machine	= atv2g_init,
MACHINE_END
