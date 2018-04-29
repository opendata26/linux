/*
 * Copyright (C) 2018 Craig Tatlor
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG 1
#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct ls052t3sx02_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct backlight_device *backlight;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;

	bool prepared;
	bool enabled;

	const struct drm_display_mode *mode;
};

static inline struct ls052t3sx02_panel *to_ls052t3sx02_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ls052t3sx02_panel, base);
}

static void panel_dcs_send(struct ls052t3sx02_panel *panel, const u8 payload,
			   const void *data, int len)
{
	mipi_dsi_dcs_write(panel->dsi, payload, data, len);
}


#define panel_dcs_write(intf, payload, cmd...) \
({\
	static const u8 seq[] = { cmd };\
	panel_dcs_send(intf, payload, seq, ARRAY_SIZE(seq));\
	})

/*
15: DCS_WRITE1
05: DCS_WRITE
39: DCS_LWRITE

dsi_lp_mode
somc,mdss-dsi-init-command:

  D
  T  L        W     D
  Y  A     A  A     L
  P  S  V  C  I     E
  E  T  C  K  T     N
  23 01 00 00 00 00 02 B0 04
  23 01 00 00 00 00 02 00 00
  23 01 00 00 00 00 02 00 00
  23 01 00 00 00 00 02 D6 01
  29 01 00 00 00 00 03 C0 0F 0F
  29 01 00 00 00 00 03 EC 00 10
  
  29 01 00 00 00 00 19 C7 \
     05 19 22 2B 38 51 41 50 5C 64 6B 74 \
     0F 23 2B 32 3F 52 44 55 61 69 70 77
  
  29 01 00 00 00 00 19 C8 \
     03 18 21 2B 38 51 42 4F 5D 65 6C 74 \
     0D 22 2A 32 3E 52 41 54 5D 66 6D 77
  
  29 01 00 00 00 00 19 C9 \
     00 15 1E 28 36 50 42 50 5E 66 6D 74 \
     0A 1F 27 2F 3D 51 41 55 5E 67 6E 77
  
  05 01 00 00 00 00 01 11
 
*/
static int ls052t3sx02_panel_init(struct ls052t3sx02_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	int ret;
	
	//mipi_dsi_generic_write(dsi, (u8[]){ 0xb0, 0x04 }, 2);
	//mipi_dsi_dcs_soft_reset(dsi);

	//msleep(10);

	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){ 0x11 }, 1);
	if (ret < 0)
		return ret;

	msleep(200);
	ret = mipi_dsi_dcs_write_buffer(dsi, (u8[]){ 0x29 }, 1);
	return 0;
}

/*
dsi_lp_mode
qcom,mdss-dsi-on-command:
  05 01 00 00 28   00 01  29                ; MIPI_DCS_SET_DISPLAY_ON
 */
static int ls052t3sx02_panel_on(struct ls052t3sx02_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	int ret;


	return 0;
}

/*
dsi_hs_mode
qcom,mdss-dsi-off-command:
  15 01 00 00 00   00 02   FF 10
  05 01 00 00 00   00 01   28               ; MIPI_DCS_SET_DISPLAY_OFF
  05 01 00 00 64   00 01   10               ; MIPI_DCS_ENTER_SLEEP_MODE
 */
static int ls052t3sx02_panel_off(struct ls052t3sx02_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	int ret;

	msleep(100);

	return 0;
}


static int ls052t3sx02_panel_disable(struct drm_panel *drm_panel)
{
	struct ls052t3sx02_panel *panel = to_ls052t3sx02_panel(drm_panel);

	if (!panel->enabled)
		return 0;

	dev_dbg(drm_panel->dev, "disable\n");

	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(panel->backlight);
	}

	panel->enabled = false;

	return 0;
}

static int ls052t3sx02_panel_unprepare(struct drm_panel *drm_panel)
{
	struct ls052t3sx02_panel *panel = to_ls052t3sx02_panel(drm_panel);
	int ret;

	if (!panel->prepared)
		return 0;

	dev_dbg(drm_panel->dev, "unprepare\n");

	ret = ls052t3sx02_panel_off(panel);
	if (ret < 0) {
		dev_err(drm_panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	regulator_disable(panel->supply);

	panel->prepared = false;

	return 0;
}

static int ls052t3sx02_panel_prepare(struct drm_panel *drm_panel)
{
	struct ls052t3sx02_panel *panel = to_ls052t3sx02_panel(drm_panel);
	int ret, i, ptr = 0;
	char id[7];
	char outbuf[128];

	//if (panel->prepared)
		return 0;

	dev_dbg(drm_panel->dev, "prepare\n");

	if (panel->reset_gpio) {
	//	gpiod_set_value(panel->reset_gpio, 0);
		msleep(250);
	}

	ret = regulator_enable(panel->supply);
	if (ret < 0)
		return ret;
	
	msleep(300);

	if (panel->reset_gpio) {
	//	gpiod_set_value(panel->reset_gpio, 1);
		msleep(250);
	}
	/*
	mipi_dsi_dcs_read(panel->dsi, 0xa1, id, 7);

	printk("id 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", id[0], id[1], id[2],
							id[3], id[4], id[5],
							id[6]);	

	*/
	ret = ls052t3sx02_panel_init(panel);
	if (ret < 0) {
		dev_err(drm_panel->dev, "failed to init panel: %d\n", ret);
		goto poweroff;
	}



	panel->prepared = true;

	return 0;
poweroff:
	regulator_disable(panel->supply);

	return ret;
}


static int ls052t3sx02_panel_enable(struct drm_panel *drm_panel)
{
	struct ls052t3sx02_panel *panel = to_ls052t3sx02_panel(drm_panel);

	//if (panel->enabled)
		return 0;

	dev_dbg(drm_panel->dev, "enable\n");

	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(panel->backlight);
	}

	panel->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
		.clock = 149074,
		.hdisplay = 1080,
		.hsync_start = 1080 + 4,
		.hsync_end = 1080 + 128 + 4,
		.htotal = 1080 + 128 + 4 + 76,
		.vdisplay = 1920,
		.vsync_start = 1920 + 2,
		.vsync_end = 1920 + 4 + 2,
		.vtotal = 1920 + 4 + 2 + 3,
		.vrefresh = 60,
};

static int ls052t3sx02_panel_get_modes(struct drm_panel *drm_panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(drm_panel->drm, &default_mode);
	if (!mode) {
		dev_err(drm_panel->drm->dev, "failed to add mode %ux%ux@%u\n",
				default_mode.hdisplay, default_mode.vdisplay,
				default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(drm_panel->connector, mode);

	drm_panel->connector->display_info.width_mm = 64;
	drm_panel->connector->display_info.height_mm = 114;

	return 1;
}

static const struct drm_panel_funcs ls052t3sx02_panel_funcs = {
		.disable = ls052t3sx02_panel_disable,
		.unprepare = ls052t3sx02_panel_unprepare,
		.prepare = ls052t3sx02_panel_prepare,
		.enable = ls052t3sx02_panel_enable,
		.get_modes = ls052t3sx02_panel_get_modes,
};

static const struct of_device_id ls052t3sx02_of_match[] = {
		{ .compatible = "sharp,ls052t3sx02", },
		{ }
};
MODULE_DEVICE_TABLE(of, ls052t3sx02_of_match);

static int ls052t3sx02_panel_add(struct ls052t3sx02_panel *panel)
{
	struct device *dev = &panel->dsi->dev;
	struct device_node *np;
	int ret;
	return 0;
	panel->mode = &default_mode;

	panel->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(panel->supply))
		return PTR_ERR(panel->supply);

	//panel->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(panel->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(panel->reset_gpio));
		panel->reset_gpio = NULL;
	} else {
		//gpiod_direction_output(panel->reset_gpio, 1);
	}

	np = of_parse_phandle(dev->of_node, "backlight", 0);
	if (np) {
		panel->backlight = of_find_backlight_by_node(np);
		of_node_put(np);

		if (!panel->backlight)
			return -EPROBE_DEFER;
	}
	if (panel->backlight) {
		panel->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(panel->backlight);
	}
	drm_panel_init(&panel->base);
	panel->base.funcs = &ls052t3sx02_panel_funcs;
	panel->base.dev = &panel->dsi->dev;

	ret = drm_panel_add(&panel->base);
	if (ret < 0)
		goto put_backlight;

	return 0;

	put_backlight:
	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return ret;
}

static void ls052t3sx02_panel_del(struct ls052t3sx02_panel *panel)
{
	if (panel->base.dev)
		drm_panel_remove(&panel->base);

	if (panel->backlight)
		put_device(&panel->backlight->dev);
}

static int ls052t3sx02_panel_probe(struct mipi_dsi_device *dsi)
{
	struct ls052t3sx02_panel *panel;
	int ret;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			MIPI_DSI_MODE_VIDEO_HSE;
	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel) {
		return -ENOMEM;
	}

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;

	ret = ls052t3sx02_panel_add(panel);
	if (ret < 0) {
		return ret;
	}

	return mipi_dsi_attach(dsi);
}

static int ls052t3sx02_panel_remove(struct mipi_dsi_device *dsi)
{
	struct ls052t3sx02_panel *panel = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = ls052t3sx02_panel_disable(&panel->base);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", ret);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	drm_panel_detach(&panel->base);
	ls052t3sx02_panel_del(panel);

	return 0;
}

static void ls052t3sx02_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct ls052t3sx02_panel *panel = mipi_dsi_get_drvdata(dsi);

	ls052t3sx02_panel_disable(&panel->base);
}

static struct mipi_dsi_driver ls052t3sx02_panel_driver = {
	.driver = {
		.name = "panel-sharp-ls052t3sx02",
		.of_match_table = ls052t3sx02_of_match,
	},
	.probe = ls052t3sx02_panel_probe,
	.remove = ls052t3sx02_panel_remove,
	.shutdown = ls052t3sx02_panel_shutdown,
};
module_mipi_dsi_driver(ls052t3sx02_panel_driver);

MODULE_AUTHOR("Craig Tatlor <ctatlor97@gmail.com>");
MODULE_DESCRIPTION("Sharp LS052T3SX02 1080p panel driver");
MODULE_LICENSE("GPL v2");
