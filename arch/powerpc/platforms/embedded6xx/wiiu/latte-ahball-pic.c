/*
 * arch/powerpc/platforms/embedded6xx/wiiu/latte-ahball-pic.c
 *
 * Nintendo Wii U "Latte" interrupt controller support.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 */
#define DRV_MODULE_NAME "latte-ahball-pic"
#define pr_fmt(fmt) DRV_MODULE_NAME ": " fmt

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/io.h>

#define LATTE_AHBALL_NR_IRQS    32

/*
 * Each interrupt has a corresponding bit in both
 * the Interrupt Cause (ICR) and Interrupt Mask (IMR) registers.
 *
 * Enabling/disabling an interrupt line involves asserting/clearing
 * the corresponding bit in IMR. ACK'ing a request simply involves
 * asserting the corresponding bit in ICR.
 */

#define LATTE_AHBALL_ICR	0x00
#define LATTE_AHBALL_IMR	0x08


/* IRQ chip hooks */

static void latte_ahball_pic_mask_and_ack(struct irq_data *d) {
	int irq = irqd_to_hwirq(d);
	void __iomem *io_base = irq_data_get_irq_chip_data(d);
	u32 mask = 1 << irq;

	out_be32(io_base + LATTE_AHBALL_ICR, mask);
	clrbits32(io_base + LATTE_AHBALL_IMR, mask);
}

static void latte_ahball_pic_ack(struct irq_data *d) {
	int irq = irqd_to_hwirq(d);
	void __iomem *io_base = irq_data_get_irq_chip_data(d);

	out_be32(io_base + LATTE_AHBALL_ICR, 1 << irq);
}

static void latte_ahball_pic_mask(struct irq_data *d) {
	int irq = irqd_to_hwirq(d);
	void __iomem *io_base = irq_data_get_irq_chip_data(d);

	clrbits32(io_base + LATTE_AHBALL_IMR, 1 << irq);
}

static void latte_ahball_pic_unmask(struct irq_data *d) {
	int irq = irqd_to_hwirq(d);
	void __iomem *io_base = irq_data_get_irq_chip_data(d);

	setbits32(io_base + LATTE_AHBALL_IMR, 1 << irq);
}


static struct irq_chip latte_ahball_pic = {
	.name			= "latte-ahball-pic",
	.irq_ack		= latte_ahball_pic_ack,
	.irq_mask_ack	= latte_ahball_pic_mask_and_ack,
	.irq_mask		= latte_ahball_pic_mask,
	.irq_unmask		= latte_ahball_pic_unmask,
};

/*	Domain Ops
 *
 */

static int latte_ahball_pic_match(struct irq_domain *h, struct device_node *node, enum irq_domain_bus_token bus_token) {
	if (h->fwnode == &node->fwnode) {
		printk("latte-ahball-pic: %s IRQ matches with this driver\n", node->name);
		return 1;
	}
	return 0;
}

static int latte_ahball_pic_alloc(struct irq_domain *h, unsigned int virq, unsigned int nr_irqs, void *arg) {
	//See espresso-pic for slight elaboration
	struct irq_fwspec* fwspec = (struct irq_fwspec*)arg;
	irq_set_chip_data(virq, h->host_data);
	irq_set_status_flags(virq, IRQ_LEVEL);
	irq_set_chip_and_handler(virq, &latte_ahball_pic, handle_level_irq);
	irq_domain_set_hwirq_and_chip(h, virq, fwspec->param[0], &latte_ahball_pic, h->host_data);
	return 0;
}

static void latte_ahball_pic_free(struct irq_domain *h, unsigned int virq, unsigned int nr_irqs) {
	printk("latte-ahball-pic: free\n");
}

const struct irq_domain_ops latte_ahball_pic_ops = {
	.match = latte_ahball_pic_match,
	.alloc = latte_ahball_pic_alloc,
	.free = latte_ahball_pic_free,
};

/*	Determinate if there are interrupts pending
 *
 */

//Store irq domain for latte_ahball_pic_get_irq (the function gets no arguments)
static struct irq_domain *latte_ahball_irq_host;

unsigned int latte_ahball_pic_get_irq(void) {
	int irq;
	u32 irq_status;
	
	void __iomem *io_base = (void __iomem*)latte_ahball_irq_host->host_data;
	
	irq_status = in_be32(io_base + LATTE_AHBALL_ICR) & in_be32(io_base + LATTE_AHBALL_IMR);

	if (irq_status == 0)
		return 0;	//No IRQs pending

	//Find the first IRQ
	irq = __ffs(irq_status);
	
	//Return the virtual IRQ
	return irq_linear_revmap(latte_ahball_irq_host, irq);
}

/*	Cascade IRQ handler
 *
 */

static void latte_ahball_pic_irq_cascade(struct irq_desc *desc) {
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;

	raw_spin_lock(&desc->lock);
	chip->irq_mask(&desc->irq_data); /* IRQ_LEVEL */
	raw_spin_unlock(&desc->lock);
	
	virq = latte_ahball_pic_get_irq();
	if (virq)
		generic_handle_irq(virq);
	else
		pr_err("latte-ahball-pic: spurious interrupt!\n");

	raw_spin_lock(&desc->lock);
	chip->irq_ack(&desc->irq_data); /* IRQ_LEVEL */
	if (!irqd_irq_disabled(&desc->irq_data) && chip->irq_unmask)
		chip->irq_unmask(&desc->irq_data);
	raw_spin_unlock(&desc->lock);
}


/*	Init function
 *
 */

struct irq_domain *latte_ahball_pic_init(struct device_node *np) {
	struct irq_domain *irq_domain;
	struct resource res;
	void __iomem *io_base;
	int retval;

	retval = of_address_to_resource(np, 0, &res);
	if (retval) {
		pr_err("no io memory range found\n");
		return NULL;
	}
	io_base = ioremap(res.start, resource_size(&res));
	if (IS_ERR(io_base)) {
		pr_err("ioremap failed\n");
		return NULL;
	}

	pr_info("controller at 0x%08x mapped to 0x%p\n", res.start, io_base);

	// mask and ack all IRQs
	out_be32(io_base + LATTE_AHBALL_IMR, 0);
	out_be32(io_base + LATTE_AHBALL_ICR, 0xffffffff);
	
	irq_domain = irq_domain_add_linear(np, LATTE_AHBALL_NR_IRQS, &latte_ahball_pic_ops, io_base);

	if (!irq_domain) {
		pr_err("latte-ahball-pic: failed to allocate irq_domain\n");
		iounmap(io_base);
		return NULL;
	}

	return irq_domain;
}

/*	Probe function
 *
 */

void latte_ahball_pic_probe(void) {
	struct irq_domain *host;
	struct device_node *np;
	const u32 *interrupts;
	int cascade_virq;

	for_each_compatible_node(np, NULL, "nintendo,latte-ahball-pic") {
		interrupts = of_get_property(np, "interrupts", NULL);
		if (interrupts) {
			host = latte_ahball_pic_init(np);
			BUG_ON(!host);
			cascade_virq = irq_of_parse_and_map(np, 0);
			irq_set_handler_data(cascade_virq, host);
			irq_set_chained_handler(cascade_virq, latte_ahball_pic_irq_cascade);
			latte_ahball_irq_host = host;
			break;
		}
	}
}
