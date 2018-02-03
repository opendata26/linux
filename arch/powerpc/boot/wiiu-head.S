/*
 * arch/powerpc/boot/wiiu-head.S
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

#include "ppc_asm.h"

/* Unlike its Wii counterpart, this code makes
 * *several* assumptions about the state of the hardware.
 * We basically expect to be running straight after the
 * Espresso BootROM.
 */

.text
.global _zimage_start
_zimage_start:
	/* Clear out all BATs. MMU is already off. */
	li r8, 0
	/*li r9, 0xBABE@l
	stw r9, 4(r8)
	dcbf 0, r8
	b .*/
	mtspr 0x210, r8	/* IBAT0U */
	mtspr 0x212, r8	/* IBAT1U */
	mtspr 0x214, r8	/* IBAT2U */
	mtspr 0x216, r8	/* IBAT3U */
	mtspr 0x218, r8	/* DBAT0U */
	mtspr 0x21a, r8	/* DBAT1U */
	mtspr 0x21c, r8	/* DBAT2U */
	mtspr 0x21e, r8	/* DBAT3U */

	mtspr 0x230, r8	/* IBAT4U */
	mtspr 0x232, r8	/* IBAT5U */
	mtspr 0x234, r8	/* IBAT6U */
	mtspr 0x236, r8	/* IBAT7U */
	mtspr 0x238, r8	/* DBAT4U */
	mtspr 0x23a, r8	/* DBAT5U */
	mtspr 0x23c, r8	/* DBAT6U */
	mtspr 0x23e, r8	/* DBAT7U */

	/* wiiu.dts has a lot more than 64MB listed, but this should be enough
	 * to run the wrapper without a problem.
	 */
	/* 64MB, starting at 0x28000000... */
	/*lis r8, 0x280007ff@ha
	ori r8, r8, 0x280007ff@l
	mtspr 0x218, r8*/	/* DBAT0U */
	/*mtspr 0x210, r8*/	/* IBAT0U */

	/* mapped into physical 0x28000000; read-write; normal caching. */
	/*lis r8, 0x28000002@ha
	ori r8, r8, 0x28000002@l
	mtspr 0x219, r8*/	/* DBAT0L */
	/*mtspr 0x211, r8*/	/* IBAT0L */

	/* MEM1 */
	/* 32MB, starting at 0x00000000... */
	li r8, 0x000003ff@l
	mtspr 0x21a, r8	/* DBAT1U */
	mtspr 0x212, r8	/* IBAT1U */

	/* mapped into physical 0x00000000; read-write; normal caching. */
	li r8, 0x00000002@l
	mtspr 0x21b, r8	/* DBAT1L */
	mtspr 0x213, r8	/* IBAT1L */

	/* Debug area */
	/* 64MB, starting at 0x89200000... */
	lis r8, 0x892007ff@ha
	ori r8, r8,0x89200007ff@l
	mtspr 0x21c, r8	/* DBAT2U */
	mtspr 0x214, r8	/* IBAT2U */

	lis r8, 0x89200002@ha
	ori r8, r8,0x8920000002@l
	mtspr 0x21d, r8	/* DBAT2L */
	mtspr 0x215, r8	/* IBAT2L */

	/* Turn on BAT 4 through 7 */
	mfspr r8, 0x3f3	/* HID4 */
	oris r8, r8, 0x0200	/* Bit 6 (from left): SBE */
	mtspr 0x3f3, r8

	/* Turn on and invalidate caches */
	mfspr r8, 0x3f0	/* HID0 */
	/* HID0_ICE | HID0_ICFI | HID0_DCE | HID0_DCFI */
	ori r8, r8, (1 << 15) | (1 << 11) | (1 << 14) | (1 << 10)
	mtspr 0x3f0, r8
	isync

	/* Arguments! */
	li r3, 0
	li r4, 0
	li r5, 0

	/* Get link register to deduce our base address */
	bl get_lr
get_lr:
	mflr r8
	/* Add offset between get_lr and done */
	/* Gets absolute address of done */
	addi r8, r8, done - get_lr

	/* We like to set our own MSR around here. */
	/* MSR_DR | MSR_IR | MSR_FP */
	/* Data/instruction MMUs, FPU because why not */
	ori r9, r3, (1 << 4) | (1 << 5) | (1 << 13)

	/* Apply our new MSR, starting execution at "done" */
	mtsrr0 r8
	mtsrr1 r9
	sync
	rfi
done:
	b _zimage_start_lib
