// SPDX-License-Identifier: GPL-2.0+

#include <asm/global_data.h>
#include <console.h>
#include <env.h>
#include <vsprintf.h>

DECLARE_GLOBAL_DATA_PTR;

void set_dfu_alt_info(char *interface, char *devstr)
{
	/* Keep DFU transfers clear of the framebuffer, initial stack and U-Boot. */
	static const char payload[] = "payload ram 0x09000000 0x01000000";
#ifdef CONFIG_CONSOLE_RECORD
	char buf[96];

	/*
	 * There is no serial console on N72AP, so expose the recorded console
	 * output as a read-only "log" alternate that can be uploaded over DFU.
	 */
	console_record_reset_enable();
	snprintf(buf, sizeof(buf), "%s;log ram 0x%lx 0x%lx", payload,
		 (ulong)gd->console_out.start,
		 (ulong)(gd->console_out.end - gd->console_out.start));
	env_set("dfu_alt_info", buf);
#else
	env_set("dfu_alt_info", payload);
#endif
}
