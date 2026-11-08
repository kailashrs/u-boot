// SPDX-License-Identifier: GPL-2.0+

#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-s5l87xx/clk.h>
#include <console.h>
#include <env.h>
#include <time.h>
#include <vsprintf.h>
#include <linux/delay.h>

DECLARE_GLOBAL_DATA_PTR;

#define N72_TIMER		0x3c700000
#define N72_TIMER64_HIGH	(N72_TIMER + 0x80)
#define N72_TIMER64_LOW		(N72_TIMER + 0x84)
#define N72_TIMER64_CTRL	(N72_TIMER + 0x88)

static u64 n72_timer64_read(void)
{
	u32 hi, lo;

	do {
		hi = readl(N72_TIMER64_HIGH);
		lo = readl(N72_TIMER64_LOW);
	} while (hi != readl(N72_TIMER64_HIGH));
	return ((u64)hi << 32) | lo;
}

/*
 * Start the 64-bit free-running counter that Linux uses as its clocksource.
 * iBSS leaves it stopped on N72. The sequence and the clock gate (openiBoot
 * gate 0x13) follow openiBoot's plat-s5l8720 timer_init_rtc().
 */
static void n72_timer64_start(void)
{
	static const struct { u8 reg; u32 bits; } gate13[] = {
		{ 1, 0x1f800020 }, { 2, 0x00000060 }, { 4, 0x00c0007f },
	};
	u64 t0, t1;
	ulong us;
	int i, b;

	for (i = 0; i < ARRAY_SIZE(gate13); i++)
		for (b = 0; b < 32; b++)
			if (gate13[i].bits & BIT(b))
				s5l87xx_enable_clkgate_bit(gate13[i].reg, b);

	t0 = n72_timer64_read();
	udelay(1000);
	if (n72_timer64_read() == t0) {
		writel(0xa, N72_TIMER64_CTRL);
		writel(0xffffffff, N72_TIMER + 0x90);
		writel(0xffffffff, N72_TIMER + 0x8c);
		writel(0xffffffff, N72_TIMER + 0x98);
		writel(0xffffffff, N72_TIMER + 0x94);
		writel(0x18010, N72_TIMER64_CTRL);
		printf("n72: started 64-bit timer\n");
	}

	/* Measure its rate against U-Boot's own microsecond timer. */
	t0 = n72_timer64_read();
	us = timer_get_us();
	udelay(100000);
	t1 = n72_timer64_read();
	us = timer_get_us() - us;
	printf("n72: 64-bit timer %llu ticks in %lu us (~%llu Hz)\n",
	       t1 - t0, us, us ? (t1 - t0) * 1000000ULL / us : 0);
}

int board_init(void)
{
	n72_timer64_start();
	return 0;
}

void set_dfu_alt_info(char *interface, char *devstr)
{
	/* Keep DFU transfers clear of the framebuffer, initial stack and U-Boot. */
	static const char payload[] = "payload ram 0x09000000 0x01000000";
#ifdef CONFIG_CONSOLE_RECORD
	char buf[160];

	/*
	 * There is no serial console on N72AP, so expose the recorded console
	 * output as a read-only "log" alternate that can be uploaded over DFU.
	 */
	/* Keep what was recorded during boot, including driver messages. */
	gd->flags |= GD_FLG_RECORD;
	/*
	 * "dram" covers all 128 MiB so leftovers from a previous boot (an old
	 * console record, the kernel's ramoops area) can be read back. It is
	 * for uploads only: a download into it would overwrite U-Boot.
	 */
	snprintf(buf, sizeof(buf), "%s;log ram 0x%lx 0x%lx;dram ram 0x08000000 0x08000000", payload,
		 (ulong)gd->console_out.start,
		 (ulong)(gd->console_out.end - gd->console_out.start));
	env_set("dfu_alt_info", buf);
#else
	env_set("dfu_alt_info", payload);
#endif
}
