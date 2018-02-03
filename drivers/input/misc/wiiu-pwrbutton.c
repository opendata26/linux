/*
 * Wii U power button input
 * This module is only for testing purposes
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/twl.h>
#include <linux/delay.h>
#include <linux/of_irq.h>

extern void __iomem* wiiu_udbg_leak_iomem(void);
extern void wiiu_udbg_putc(char c);
extern struct irq_desc *irq_to_desc(unsigned int irq);

static irqreturn_t wiiu_powerbutton_irq(int irq, void *ptr)
{
	//Do not report input, only print the event for testing
	printk("WII U POWER BUTTON PRESSED\n");
	//udelay(5*1000*1000);
	return IRQ_HANDLED;
}

extern int virq_debug_show(void *private);

static int wiiu_pwrbutton_probe(struct platform_device *pdev)
{
	int err;
	int irq;
	struct device_node *np;

	printk("wiiu_pwrbutton_probe\n");

	np = of_find_compatible_node(NULL, NULL, "nintendo,wiiu-pwrbutton");

	irq = irq_of_parse_and_map(np, 0);
	printk("irq: %d\n", irq);

	err = request_irq(irq, wiiu_powerbutton_irq, 0, "wiiu_pwrbutton", np);
	if (err < 0)
		printk("request_irq failed: %d\n", err);


	//virq_debug_show(NULL);

	//printk("TRY POWER BUTTON NOW\n");

	//udelay(5*1000*1000);

	return 0;
}

static const struct of_device_id wiiu_pwrbutton_dt_match_table[] = {
       { .compatible = "nintendo,wiiu-pwrbutton" },
       {},
};
MODULE_DEVICE_TABLE(of, wiiu_pwrbutton_dt_match_table);

static struct platform_driver wiiu_pwrbutton_driver = {
	.driver = {
		.name = "wiiu_pwrbutton",
		.of_match_table = of_match_ptr(wiiu_pwrbutton_dt_match_table),
	},
	.probe = wiiu_pwrbutton_probe,
};
module_platform_driver(wiiu_pwrbutton_driver);

MODULE_DESCRIPTION("Wii U Power Button");
MODULE_LICENSE("GPL");
