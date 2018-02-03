/*
 * arch/powerpc/plaforms/wiiu_udbg.c
 *
 * Udbg backend to send messages over Wii U's IPC
 * Copyright (C) 2017 Ash Logan <quarktheawesome@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

 #include <mm/mmu_decl.h>

 #include <asm/io.h>
 #include <asm/prom.h>
 #include <asm/udbg.h>
 #include <asm/fixmap.h>

/* PROTOCOL:
 * PPC writes char into PPCMSG_COMPAT
 * PPC sets X1; signalling the ARM
 * ARM does whatever it needs with the character
 * ARM clears X1; signalling the PPC
 * repeat
 */

//TODO get this from the dts
#define WIIU_IPC_IO_BASE (phys_addr_t)0x0d800000

static void __iomem* ipc_io_base;
#define LT_IPC_PPCMSG_COMPAT (u32 __iomem*)(ipc_io_base + 0)
#define LT_IPC_PPCCTRL_COMPAT (u32 __iomem*)(ipc_io_base + 4)
#define CTRL_X1 0x1

void __iomem* wiiu_udbg_leak_iomem(void) {
	return ipc_io_base;
}

bool wiiu_udbg_ready(void) {
	return !(in_be32(LT_IPC_PPCCTRL_COMPAT) & CTRL_X1);
}

void wiiu_udbg_putc(char ch) {
	u32 ctrl;
	if (!ipc_io_base) return;

	while (!wiiu_udbg_ready()) {
		barrier();
	}

	out_be32(LT_IPC_PPCMSG_COMPAT, (u32)ch);
	ctrl = in_be32(LT_IPC_PPCCTRL_COMPAT);
	ctrl |= CTRL_X1;
	out_be32(LT_IPC_PPCCTRL_COMPAT, ctrl);
}

#ifdef CONFIG_PPC_EARLY_DEBUG_WIIU

void __init udbg_init_wiiu(void) {
	void __iomem* early_debug_area;

	early_debug_area = (void __iomem*)__fix_to_virt(FIX_EARLY_DEBUG_BASE);
	ipc_io_base = early_debug_area;

	udbg_putc = wiiu_udbg_putc;

	/* Set the BAT so that our addresses
	 * are preserved when the MMU turns on.
	 */
	setbat(1, (unsigned long)early_debug_area, WIIU_IPC_IO_BASE, 128*1024, PAGE_KERNEL_NCG);
}

#endif /* CONFIG_PPC_EARLY_DEBUG_WIIU */
