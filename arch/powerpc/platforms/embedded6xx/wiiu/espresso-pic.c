/*
 * arch/powerpc/platforms/embedded6xx/wiiu/espresso-pic.c
 *
 * Nintendo Wii U "Espresso" interrupt controller support
 * http://wiiubrew.org/wiki/Hardware/Processor_Interface
 *
 * Instead of using COS custom IRQ remapping, the normal IRQ mapping is used instead:
 *
 *  IRQ         Description
 * -------------------------------------------
 * 	0			Error
 * 	1			Unused 
 * 	2			Unused
 * 	3			Audio Interface (TV)
 * 	4			Unused
 * 	5			DSP Accelerator
 * 	6			DSP
 * 	7			DSP DMA
 * 	8			Unused
 * 	9			Unused
 * 	10			GPIPPC (?)
 * 	11			Unused
 * 	12			Audio Interface (Gamepad)
 * 	13			I2C
 * 	14			Unused
 * 	15			Unused
 * 	16			Unused
 * 	17			Unused
 * 	18			Unused
 * 	19			Unused
 * 	20			Unused
 * 	21			Unused
 * 	22			Unused
 * 	23			GX2
 * 	24			Latte IRQ Controller
 * 	25			Unused
 * 	26			IPC (CPU2)
 * 	27			Unused
 * 	28			IPC (CPU1)
 * 	29			Unused
 * 	30			IPC (CPU0)
 * 	31			Unused
 */

#define DRV_MODULE_NAME "espresso-pic"
#define pr_fmt(fmt) DRV_MODULE_NAME ": " fmt

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/io.h>

#define ESPRESSO_NR_IRQS 32

#define PI_INTSR_GLOBAL 0x00
#define PI_INTMR_GLOBAL 0x04

/*	Chip stuff
 *
 */

static void espresso_pic_mask_and_ack(struct irq_data* d) {
	int irq = irqd_to_hwirq(d);
	void __iomem* io_base = irq_data_get_irq_chip_data(d);
	u32 mask = 1 << irq;

	clrbits32(io_base + PI_INTMR_GLOBAL, mask);
	out_be32(io_base + PI_INTSR_GLOBAL, mask);
}

static void espresso_pic_ack(struct irq_data* d) {
	int irq = irqd_to_hwirq(d);
	void __iomem* io_base = irq_data_get_irq_chip_data(d);
	
	out_be32(io_base + PI_INTSR_GLOBAL, 1 << irq);
}

static void espresso_pic_mask(struct irq_data* d) {
	int irq = irqd_to_hwirq(d);
	void __iomem* io_base = irq_data_get_irq_chip_data(d);

	clrbits32(io_base + PI_INTMR_GLOBAL, 1 << irq);
}

static void espresso_pic_unmask(struct irq_data* d) {
	int irq = irqd_to_hwirq(d);
	void __iomem* io_base = irq_data_get_irq_chip_data(d);

	setbits32(io_base + PI_INTMR_GLOBAL, 1 << irq);
}

static struct irq_chip espresso_pic_chip = {
	.name			= "espresso-pic",
	.irq_ack		= espresso_pic_ack,
	.irq_mask_ack	= espresso_pic_mask_and_ack,
	.irq_mask		= espresso_pic_mask,
	.irq_unmask		= espresso_pic_unmask,
};

/*	Domain Ops
 *
 */

static int espresso_pic_match(struct irq_domain *h, struct device_node *node, enum irq_domain_bus_token bus_token) {
	if (h->fwnode == &node->fwnode) {
		printk("espresso-pic: %s IRQ matches with this driver\n", node->name);
		return 1;
	}
	return 0;
}

static int espresso_pic_alloc(struct irq_domain *h, unsigned int virq, unsigned int nr_irqs, void *arg) {
	//hacky; but it works. Brilliantly.
	struct irq_fwspec* fwspec = (struct irq_fwspec*)arg;

	irq_set_chip_data(virq, h->host_data);
	irq_set_status_flags(virq, IRQ_LEVEL);
	irq_set_chip_and_handler(virq, &espresso_pic_chip, handle_level_irq);
	//Here's the other end of that wonderful hack
	irq_domain_set_hwirq_and_chip(h, virq, fwspec->param[0], &espresso_pic_chip, h->host_data);
	return 0;
}

static void espresso_pic_free(struct irq_domain *h, unsigned int virq, unsigned int nr_irqs) {
	printk("espresso-pic: free\n");
}

const struct irq_domain_ops espresso_pic_ops = {
	.match = espresso_pic_match,
	.alloc = espresso_pic_alloc,
	.free = espresso_pic_free,
};

/*	Determinate if there are interrupts pending
 *
 */

//Store irq domain for espresso_pic_get_irq (the function gets no arguments)
static struct irq_domain *espresso_irq_domain;

unsigned int espresso_pic_get_irq(void)
{
	int irq;
	u32 irq_status;

	void __iomem* io_base = (void __iomem*)espresso_irq_domain->host_data;

	irq_status = in_be32(io_base + PI_INTSR_GLOBAL) & in_be32(io_base + PI_INTMR_GLOBAL);

	if (irq_status == 0)
		return 0;	//No IRQs pending

	//Find the first IRQ
	irq = __ffs(irq_status);
		
	//Return the virtual IRQ
	return irq_linear_revmap(espresso_irq_domain, irq);
}

/*	Init functions
 *
 */

static struct irq_domain* espresso_pic_init(struct device_node* np) {
	struct irq_domain *irq_domain;
	struct resource res;
	void __iomem *io_base;
	int retval;

	retval = of_address_to_resource(np, 0, &res);
	BUG_ON(retval);
	
	io_base = ioremap(res.start, resource_size(&res));
	BUG_ON(IS_ERR(io_base));

	pr_info("espresso-pic: controller at 0x%08x mapped to 0x%p\n", res.start, io_base);

	//__espresso_pic_quiesce(io_base);

	out_be32(io_base + PI_INTMR_GLOBAL, 0);
	out_be32(io_base + PI_INTSR_GLOBAL, 0xFFFFFFFF);

	irq_domain = irq_domain_add_linear(np, ESPRESSO_NR_IRQS, &espresso_pic_ops, io_base);
	BUG_ON(!irq_domain);

	printk("espresso-pic: dev name: %s\n", np->name);
	printk("espresso-pic:     name: %s\n", irq_domain->name);

	//Save irq domain for espresso_pic_get_irq
	espresso_irq_domain = irq_domain;

	return irq_domain;
}

void espresso_pic_probe(void) {
	struct device_node* np;
	struct irq_domain* host;
	np = of_find_compatible_node(NULL, NULL, "nintendo,espresso-pic");
	BUG_ON(!np);

	host = espresso_pic_init(np);

	irq_set_default_host(host);

	of_node_put(np);
}
