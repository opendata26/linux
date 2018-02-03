/*
 * arch/powerpc/platforms/embedded6xx/latte-ahball-pic.h
 *
 * Nintendo Wii U "Latte" interrupt controller support
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */

#ifndef __LATTE_AHBALL_PIC_H
#define __LATTE_AHBALL_PIC_H

extern unsigned int latte_ahball_pic_get_irq(void);
extern void latte_ahball_pic_probe(void);
extern void latte_ahball_quiesce(void);

#endif
