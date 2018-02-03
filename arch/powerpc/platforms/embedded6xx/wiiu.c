/*
 * arch/powerpc/platforms/wiiu.c
 *
 * Nintendo Wii U
 * Copyright (C) 2017 Ash Logan <quarktheawesome@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

//TODO again, this code is mostly taken from GameCube.

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/kexec.h>
#include <linux/seq_file.h>
#include <linux/of_platform.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <mm/mmu_decl.h>

#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/time.h>
#include <asm/udbg.h>

#include "wiiu/latte-ahball-pic.h"
#include "wiiu/espresso-pic.h"

phys_addr_t wiiu_hole_addrs[10];
phys_addr_t wiiu_hole_sizes[10];
int wiiu_hole_count = 0;

/*	Workarounds for memory tomfoolery.
 *	Adapted from wii.c, kinda.
 */
//Pull this out of pgtable_32.c
extern void __mapin_ram_chunk(unsigned long offset, unsigned long top);

static int __init page_aligned(unsigned long x)
{
	return !(x & (PAGE_SIZE-1));
}


/*	Add in memory gaps for various calculations
*/
void __init wiiu_memory_fixups(void) {
	int i;

	struct memblock_region* p = memblock.memory.regions;

	BUG_ON(memblock.memory.cnt < 2);
	for (i = 0; i < memblock.memory.cnt; i++) {
		BUG_ON(!page_aligned(p[i].base));
	}

	//Construct a list of holes to add
	BUG_ON(memblock.memory.cnt - 1 > 10);
	for (i = 0; i < memblock.memory.cnt - 1; i++) {
		//Start of hole (top of good region)
		wiiu_hole_addrs[i] = ALIGN(p[i].base + p[i].size, PAGE_SIZE);
		//Size of hole
		wiiu_hole_sizes[i] = p[i + 1].base - wiiu_hole_addrs[i];
		//Increment hole count
		wiiu_hole_count++;
	}

	//Reserve all memory holes
	for (i = 0; i < wiiu_hole_count; i++) {
		memblock_add(wiiu_hole_addrs[i], wiiu_hole_sizes[i]);
		memblock_reserve(wiiu_hole_addrs[i], wiiu_hole_sizes[i]);
		printk("reserved memory from %08X to %08X\n", wiiu_hole_addrs[i], wiiu_hole_addrs[i] + wiiu_hole_sizes[i]);
	}

	//Join all the memblocks (in some cases the memblock may not join by reserving memory)
	if(memblock.memory.cnt != 1) {
		p[0].size = (p[memblock.memory.cnt - 1].base + p[memblock.memory.cnt - 1].size) - p[0].base;
		memblock.memory.cnt = 1;
	}

	__memblock_dump_all();

	__allow_ioremap_reserved = 1;
}

/* Map memory
 * This function assumes the memory holes list to be already ordered and that there's only one memblock
 */
void __init wiiu_map_ram() {
	int i;
	phys_addr_t map_offset;
	struct memblock_region* p = memblock.memory.regions;

	BUG_ON(memblock.memory.cnt != 1);

	//Map memory
	printk("mapping memory... (PAGE_OFFSET = %08lX)\n", PAGE_OFFSET);

	//Currently mapping offset
	map_offset = p[0].base;

	//Add end of memory
	wiiu_hole_addrs[wiiu_hole_count] = p[0].base + p[0].size;

	for (i = 0; i <= wiiu_hole_count; i++) {
		//Map from current offset to the start of the next memory hole
		__mapin_ram_chunk(map_offset, wiiu_hole_addrs[i]);
		printk("mapped memory from %08X to %08X\n", map_offset, wiiu_hole_addrs[i]);

		//Update the offset to the end of the current memory hole
		map_offset = wiiu_hole_addrs[i] + wiiu_hole_sizes[i];
	}
}

/* Add system ram resources which shouldn't be reserved to resources list
 */
void __init wiiu_add_system_ram_resources() {
	int i;
	phys_addr_t map_offset;
	struct memblock_region* p = memblock.memory.regions;

	BUG_ON(memblock.memory.cnt != 1);

	//Currently mapping offset
	map_offset = p[0].base;

	//Add end of memory
	wiiu_hole_addrs[wiiu_hole_count] = p[0].base + p[0].size;

	for (i = 0; i <= wiiu_hole_count; i++) {
		//Mark the memory region from current offset to the start of the next memory hole as System RAM
		struct resource *res = kzalloc(sizeof(struct resource), GFP_KERNEL);
		WARN_ON(!res);

		if (res) {
			res->name = "System RAM";
			res->start = map_offset;
			res->end = wiiu_hole_addrs[i];
			res->flags = IORESOURCE_SYSTEM_RAM | IORESOURCE_BUSY;
			WARN_ON(request_resource(&iomem_resource, res) < 0);
		}

		printk("marked memory from %08X to %08X as system ram\n", map_offset, wiiu_hole_addrs[i]);

		//Update the offset to the end of the current memory hole
		map_offset = wiiu_hole_addrs[i] + wiiu_hole_sizes[i];
	}
}

static void __noreturn wiiu_spin(void) {
	for (;;) {
		cpu_relax();
	}
}

static void __noreturn wiiu_halt(void) {
	local_irq_disable();
	wiiu_spin();
}

//TODO: Place code in a proper device
#define LT_IPC_IO_BASE 0x0d800000
#define LT_IPC_PPCMSG_COMPAT 0x0
#define LT_IPC_PPCCTRL_COMPAT 0x4
#define LT_CTRL_X1 0x1
#define LT_CMD_SHUTDOWN 0xCAFE0001
#define LT_CMD_RESTART 0xCAFE0002

static void __noreturn wiiu_restart(char *cmd) {
	// Map IPC registers
	void __iomem *wiiu_ipc = ioremap(LT_IPC_IO_BASE, 0x10);
	
	// Write reboot command
	out_be32(wiiu_ipc + LT_IPC_PPCMSG_COMPAT, LT_CMD_RESTART);
	
	// Set signal flag
	out_be32(wiiu_ipc + LT_IPC_PPCCTRL_COMPAT,
			 in_be32(wiiu_ipc + LT_IPC_PPCCTRL_COMPAT) | LT_CTRL_X1);
	
	// Wait for the ARM to power everything down
	wiiu_spin();
}
static void __noreturn wiiu_shutdown(void) {
	// Map IPC registers
	void __iomem *wiiu_ipc = ioremap(LT_IPC_IO_BASE, 0x10);
	
	// Write reboot command
	out_be32(wiiu_ipc + LT_IPC_PPCMSG_COMPAT, LT_CMD_SHUTDOWN);
	
	// Set signal flag
	out_be32(wiiu_ipc + LT_IPC_PPCCTRL_COMPAT,
			 in_be32(wiiu_ipc + LT_IPC_PPCCTRL_COMPAT) | LT_CTRL_X1);
	
	// Wait for the ARM to power everything down
	wiiu_spin();
}

static int __init wiiu_probe(void) {
	if (!of_machine_is_compatible("nintendo,wiiu")) {
		return 0;
	}

	pm_power_off = &wiiu_shutdown;

	return 1;
}

void wiiu_irq_init(void) {
	espresso_pic_probe();
	latte_ahball_pic_probe();
}

define_machine(wiiu) {
	.name			= "wiiu",
	.probe			= wiiu_probe,
	.restart		= wiiu_restart,
	.halt			= wiiu_halt,
	.init_IRQ		= wiiu_irq_init,
	.get_irq		= espresso_pic_get_irq,
	.calibrate_decr	= generic_calibrate_decr,
	.progress		= udbg_progress,
};

static const struct of_device_id wiiu_of_bus[] = {
	{ .compatible = "nintendo,latte", },
	{ .compatible = "simple-bus", },
	{ },
};

static int __init wiiu_device_probe(void)
{
	if (!machine_is(wiiu))
		return 0;

	of_platform_populate(NULL, wiiu_of_bus, NULL, NULL);
	return 0;
}
device_initcall(wiiu_device_probe);
