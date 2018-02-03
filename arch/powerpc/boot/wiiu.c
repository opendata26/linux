/*
 * arch/powerpc/boot/wiiu.c
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

 #include <stddef.h>
 #include "stdio.h"
 #include "types.h"
 #include "io.h"
 #include "ops.h"

BSS_STACK(8192);

void* mem2_dbg_location = (void *)0x89200000;
void mem2_dbg_write(const char* buf, int len) {
	memcpy(mem2_dbg_location, buf, len);
	mem2_dbg_location += len;
}

/* Mostly copied from gamecube.c. Obviously the GameCube is not the same
 * as the Wii U. TODO.
 */
void platform_init(unsigned int r3, unsigned int r4, unsigned int r5) {
	u32 heapsize = 16*1024*1024 - (u32)_end;
	simple_alloc_init(_end, heapsize, 32, 64);

	fdt_init(_dtb_start);

	console_ops.write = mem2_dbg_write;
}
