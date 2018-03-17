#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slimbus/slimbus.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <sound/info.h>

#include "wcd9320.h"
#include "wcd9320_registers.h"
#include "wcd-slim.h"
#include "wcd-clsh.h"

#define SLIMBUS_PRESENT_TIMEOUT 100
#define WCD9XXX_MCLK_CLK_9P6HZ 9600000
#define WCD9XXX_MCLK_CLK_12P288MHZ 12288000
#define WCD9XXX_REGISTER_START_OFFSET 0x800

#define WCD9320_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

struct wcd_slim_codec_dai_data {
	u32 rate;				/* sample rate          */
	u32 bit_width;				/* sit width 16,24,32   */
	struct list_head wcd_slim_ch_list;	/* channel list         */
	u16 grph;				/* slimbus group handle */
	unsigned long ch_mask;
	wait_queue_head_t dai_wait;
};

struct wcd9335 {
	struct device *dev;

	/* data */
	int irq;
	struct clk *ext_clk;
	struct clk *native_clk;
	int clk1_gpio;
	int reset_gpio;
	int irq_gpio;
	int mclk_rate;
	int num_of_supplies;
	struct regulator_bulk_data supplies[8];

	u8 version;
	/* slimbus specific*/
	struct slim_device *slim;
	struct slim_device slim_ifd;
	struct slim_device *slim_slave;


	struct regmap *regmap;
	struct regmap *ifd_regmap;

	struct wcd_slim_data slim_data;

	//
	struct wcd_slim_codec_dai_data  dai[1];
	struct wcd_clsh_cdc_data clsh_d;
	unsigned rx_port_value;
	unsigned rx_bias_count;
	unsigned hph_mode;
};

static int reg_read(struct slim_device *slim, unsigned int reg)
{
	struct slim_ele_access msg = {0,};
	int ret;
	u8 val;

//	pr_err("DEBUG:: %s: %x \n", __func__, reg);
	msg.start_offset = WCD9XXX_REGISTER_START_OFFSET + reg;
	msg.num_bytes = 1;
	msg.comp = NULL;
//	msg.rbuf = (void *)val;

	ret = slim_request_val_element(slim, &msg, (void *)&val, 1);
	//printk("slim read %X=%X\n", reg, val);
	return val;
}

static void reg_write(struct slim_device *slim, unsigned int reg, u8 val)
{
	struct slim_ele_access msg = {0,};
	//u8 val0, val2;
	int ret;//, ret2;

	msg.start_offset = WCD9XXX_REGISTER_START_OFFSET + reg;
	msg.num_bytes = 1;
	msg.comp = NULL;

	//printk("slim write %X=%X\n", reg, val);

	//ret2 = slim_request_val_element(slim, &msg, (void *)&val0, 1);

	ret = slim_change_val_element(slim, &msg, (void *)&val, 1);
	WARN_ON(ret);

	//usleep_range(5000, 5000);

	//ret2 = slim_request_val_element(slim, &msg, (void *)&val2, 1);

	//printk("%X reg %X->%X=%X %i %i\n", reg, val0, val, val2, ret, ret2);
}

static void reg_update(struct slim_device *slim, u16 reg, u8 mask, u8 val)
{
	u8 tmp;
	tmp = reg_read(slim, reg);
	tmp &= ~mask;
	tmp |= val;
	reg_write(slim, reg, tmp);
}

static struct regmap_config wcd9320_ifd_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
//	.can_multi_write = true,
};

#define NUM_INTERPOLATORS 7
#define TAIKO_RX_PORT_START_NUMBER	16

enum {
	AIF1_PB = 0,
	/*AIF1_CAP,
	AIF2_PB,
	AIF2_CAP,
	AIF3_PB,
	AIF3_CAP,
	AIF4_VIFEED,
	AIF4_MAD_TX, */
	NUM_CODEC_DAIS,
};


enum {
	RX_MIX1_INP_SEL_ZERO = 0,
	RX_MIX1_INP_SEL_SRC1,
	RX_MIX1_INP_SEL_SRC2,
	RX_MIX1_INP_SEL_IIR1,
	RX_MIX1_INP_SEL_IIR2,
	RX_MIX1_INP_SEL_RX1,
	RX_MIX1_INP_SEL_RX2,
	RX_MIX1_INP_SEL_RX3,
	RX_MIX1_INP_SEL_RX4,
	RX_MIX1_INP_SEL_RX5,
	RX_MIX1_INP_SEL_RX6,
	RX_MIX1_INP_SEL_RX7,
	RX_MIX1_INP_SEL_AUXRX,
};

static int taiko_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	unsigned rates[] = {8000, 16000, 32000, 48000, 96000, 192000};
	u8 rate_index = 0;
	unsigned rate = params_rate(params);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(dai->codec);
	struct snd_soc_codec *codec = dai->codec;
	struct wcd_slim_ch *ch;
	u32 j;
	u8 rx_mix1_inp;
	u16 rx_mix_1_reg_1, rx_mix_1_reg_2;
	u16 rx_fs_reg;
	u8 rx_mix_1_reg_1_val, rx_mix_1_reg_2_val;

	printk("taiko_hw_params\n");

	do {
		if (rate_index == ARRAY_SIZE(rates))
			return -EINVAL;
	} while (rate != rates[rate_index] && ++rate_index);

	list_for_each_entry(ch, &wcd->dai[dai->id].wcd_slim_ch_list, list) {
		rx_mix1_inp = ch->port + RX_MIX1_INP_SEL_RX1 - 16; //TAIKO_TX_PORT_NUMBER;
		WARN_ON(rx_mix1_inp < RX_MIX1_INP_SEL_RX1 || rx_mix1_inp > RX_MIX1_INP_SEL_RX7);

		rx_mix_1_reg_1 = TAIKO_A_CDC_CONN_RX1_B1_CTL;

		for (j = 0; j < NUM_INTERPOLATORS; j++) {
			rx_mix_1_reg_2 = rx_mix_1_reg_1 + 1;

			rx_mix_1_reg_1_val = snd_soc_read(codec, rx_mix_1_reg_1);
			rx_mix_1_reg_2_val = snd_soc_read(codec, rx_mix_1_reg_2);

			if (((rx_mix_1_reg_1_val & 0x0F) == rx_mix1_inp) ||
				(((rx_mix_1_reg_1_val >> 4) & 0x0F) == rx_mix1_inp) ||
				((rx_mix_1_reg_2_val & 0x0F) == rx_mix1_inp)) {

				rx_fs_reg = TAIKO_A_CDC_RX1_B5_CTL + 8 * j;

				printk("%s: AIF_PB DAI(%d) connected to RX%u\n", __func__, dai->id, j + 1);
				printk("%s: set RX%u sample rate to %u\n", __func__, j + 1, rate);

				snd_soc_update_bits(codec, rx_fs_reg, 0xE0, rate_index << 5);
			}
			if (j < 2)
				rx_mix_1_reg_1 += 3;
			else
				rx_mix_1_reg_1 += 2;
		}
	}
	return 0;
}

static int taiko_set_dai_fmt(struct snd_soc_dai *dai, unsigned fmt)
{
	printk("taiko_set_dai_fmt\n");

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int taiko_codec_enable_slim_chmask(struct wcd_slim_codec_dai_data *dai,
					  bool up);

static int taiko_set_channel_map(struct snd_soc_dai *dai,
				 unsigned int tx_num, unsigned int *tx_slot,
				 unsigned int rx_num, unsigned int *rx_slot)
{
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(dai->codec);

	printk("taiko_set_channel_map\n");

	if (!tx_slot || !rx_slot) {
		dev_err(wcd->dev, "Invalid tx_slot=%p, rx_slot=%p\n",
			tx_slot, rx_slot);
		return -EINVAL;
	}

	wcd_slim_init_slimslave(&wcd->slim_data, wcd->slim->laddr, tx_num, tx_slot, rx_num, rx_slot);

	/*msleep(1000);
	msleep(1000);
	msleep(1000);
	msleep(1000);
	msleep(1000);

	list_add_tail(&wcd->slim_data.rx_chs[0].list,
			      &wcd->dai[AIF1_PB].wcd_slim_ch_list);
	list_add_tail(&wcd->slim_data.rx_chs[1].list,
			      &wcd->dai[AIF1_PB].wcd_slim_ch_list);

	taiko_codec_enable_slim_chmask(wcd->dai, true);
	wcd_slim_cfg_slim_sch_rx(&wcd->slim_data, &wcd->dai->wcd_slim_ch_list, 48000, 16, &wcd->dai->grph);*/
	return 0;
}

static int taiko_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	static int prepared;
	__auto_type codec = dai->codec;
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(dai->codec);
	printk("taiko prepare %u %u\n", wcd->dai->rate, wcd->dai->bit_width);

	if (prepared)
		return 0;
	prepared = 1;

	snd_soc_update_bits(codec, TAIKO_A_CDC_CONN_RX_SB_B1_CTL, 0xff, 0x02);
	snd_soc_update_bits(codec, TAIKO_A_CDC_CONN_RX_SB_B1_CTL, 0xff, 0x0a);

	snd_soc_write(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0xD4);
	snd_soc_write(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0xD5);
	snd_soc_write(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0x55);

	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN1, 4);
	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN1, 5);
	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN2, 0);
	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN2, 4);

	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B1_CTL, 0x44);
	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B6_CTL, 0x2f);
	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B2_CTL, 0x03);
	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B3_CTL, 0x17);

	snd_soc_write(codec, TAIKO_A_RX_COM_BIAS, 0x80);

	snd_soc_write(codec, TAIKO_A_CDC_CONN_CLSH_CTL, 0x14);
	snd_soc_write(codec, TAIKO_A_CDC_CLK_RDAC_CLK_EN_CTL, 0x04);
	snd_soc_write(codec, WCD9XXX_A_RX_HPH_R_DAC_CTL, 0x40);
	snd_soc_write(codec, WCD9XXX_A_BUCK_CTRL_CCL_4, 0x50);
	snd_soc_write(codec, TAIKO_A_BUCK_CTRL_CCL_1, 0x5b);

	snd_soc_write(codec, TAIKO_A_BUCK_CTRL_CCL_3, 0x68);
	snd_soc_write(codec, TAIKO_A_BUCK_CTRL_CCL_3, 0x60);

	snd_soc_write(codec, TAIKO_A_CDC_CLSH_BUCK_NCP_VARS, 0x04);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B2_CTL, 0x01);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B2_CTL, 0x05);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B2_CTL, 0x35);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B3_CTL, 0x30);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B3_CTL, 0x3b);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B1_CTL, 0xe6);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B1_CTL, 0xa6);

	//snd_soc_write(codec, TAIKO_A_CDC_CLSH_V_PA_HD_HPH, 0x0d);
	//snd_soc_write(codec, TAIKO_A_CDC_CLSH_V_PA_MIN_HPH, 0x1d);
	//snd_soc_write(codec, TAIKO_A_CDC_CLSH_IDLE_HPH_THSD, 0x13);
snd_soc_write(codec, 0xB2F - 0x800, 0xD);
snd_soc_write(codec, 0xB31 - 0x800, 0x1D);
snd_soc_write(codec, 0xB24 - 0x800, 0x13);
snd_soc_write(codec, 0xB26 - 0x800, 0x19);
snd_soc_write(codec, 0xB2A - 0x800, 0x97);
snd_soc_write(codec, 0xB29 - 0x800, 0xAE);
snd_soc_write(codec, 0xB29 - 0x800, 0x1);
snd_soc_write(codec, 0xB29 - 0x800, 0x1C);
snd_soc_write(codec, 0xB29 - 0x800, 0x0);
snd_soc_write(codec, 0xB29 - 0x800, 0x24);
snd_soc_write(codec, 0xB29 - 0x800, 0x0);
snd_soc_write(codec, 0xB29 - 0x800, 0x25);
snd_soc_write(codec, 0xB29 - 0x800, 0x0);
snd_soc_write(codec, 0xB20 - 0x800, 0xA7);
snd_soc_write(codec, 0xB0C - 0x800, 0x1);
snd_soc_write(codec, 0xB20 - 0x800, 0xA3);
snd_soc_write(codec, 0x985 - 0x800, 0x2);
snd_soc_write(codec, 0x984 - 0x800, 0xFF);
snd_soc_write(codec, 0x981 - 0x800, 0x25);
snd_soc_write(codec, 0x983 - 0x800, 0xCA);
snd_soc_write(codec, 0x983 - 0x800, 0xC2);
snd_soc_write(codec, 0x981 - 0x800, 0xA5);
snd_soc_write(codec, 0x992 - 0x800, 0xFF);
snd_soc_write(codec, 0x9B7 - 0x800, 0xC0);
snd_soc_write(codec, 0xB0D - 0x800, 0x6);
snd_soc_write(codec, 0x9B1 - 0x800, 0xC0);
snd_soc_write(codec, 0xAB5 - 0x800, 0xA0);
snd_soc_write(codec, 0xABD - 0x800, 0xA0);
snd_soc_write(codec, 0xB01 - 0x800, 0x1);
snd_soc_write(codec, 0xB01 - 0x800, 0x0);
snd_soc_write(codec, 0xB01 - 0x800, 0x2);
snd_soc_write(codec, 0xB01 - 0x800, 0x0);
snd_soc_write(codec, 0xB0F - 0x800, 0x3);
snd_soc_write(codec, 0xAB7 - 0x800, 0x0);
snd_soc_write(codec, 0xABF - 0x800, 0x0);

regmap_write(wcd->ifd_regmap, 0x980 - 0x800, 0x3);
regmap_write(wcd->ifd_regmap, 0x840 - 0x800, 0x5);
regmap_write(wcd->ifd_regmap, 0x984 - 0x800, 0x3);
regmap_write(wcd->ifd_regmap, 0x841 - 0x800, 0x5);

snd_soc_write(codec, 0x9AB - 0x800, 0xB0);
snd_soc_write(codec, 0x985 - 0x800, 0x0);
snd_soc_write(codec, 0x994 - 0x800, 0x8);
snd_soc_write(codec, 0x983 - 0x800, 0xC6);
snd_soc_write(codec, 0x983 - 0x800, 0xCE);

	taiko_codec_enable_slim_chmask(wcd->dai, true);
	wcd_slim_cfg_slim_sch_rx(&wcd->slim_data, &wcd->dai->wcd_slim_ch_list,
					      48000, 16,
					      &wcd->dai->grph);
	return 0;
}

static struct snd_soc_dai_ops taiko_dai_ops = {
	.hw_params = taiko_hw_params,
	.prepare = taiko_prepare,
	.set_fmt = taiko_set_dai_fmt,
	.set_channel_map = taiko_set_channel_map,
	//.get_channel_map = wcd9335_get_channel_map,
};

static struct snd_soc_dai_driver taiko_dai[] = {
	{
		.name = "taiko_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9320_RATES,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FORMAT_S24_LE),
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &taiko_dai_ops,
	}
};

static int taiko_codec_enable_slim_chmask(struct wcd_slim_codec_dai_data *dai,
					  bool up)
{
	int ret = 0;
	struct wcd_slim_ch *ch;

	if (up) {
		list_for_each_entry(ch, &dai->wcd_slim_ch_list, list) {
			ret = wcd_slim_get_slave_port(ch->ch_num);
			printk("enable_chmask %u->%u\n", ch->ch_num, ret);
			if (ret < 0) {
				pr_err("%s: Invalid slave port ID: %d\n",
				       __func__, ret);
				ret = -EINVAL;
			} else {
				set_bit(ret, &dai->ch_mask);
			}
		}
	} else {
		ret = wait_event_timeout(dai->dai_wait, (dai->ch_mask == 0),
					 msecs_to_jiffies(1000));
		if (!ret) {
			pr_err("%s: Slim close tx/rx wait timeout, ch_mask:0x%lx\n",
				__func__, dai->ch_mask);
			ret = -ETIMEDOUT;
		} else {
			ret = 0;
		}
	}
	return ret;
}

static int taiko_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *kcontrol,
				     int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	struct wcd_slim_codec_dai_data *dai;

	dai = &wcd->dai[w->shift];

	printk("taiko_codec_enable_slimrx\n");

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		// wcd9335_codec_enable_int_port(dai, codec);
		(void) taiko_codec_enable_slim_chmask(dai, true);
		wcd_slim_cfg_slim_sch_rx(&wcd->slim_data, &dai->wcd_slim_ch_list,
					      dai->rate, dai->bit_width,
					      &dai->grph);
		break;
	case SND_SOC_DAPM_POST_PMD:
		// wcd9335_codec_vote_max_bw(codec, true);
		ret = wcd_slim_disconnect_port(&wcd->slim_data, &dai->wcd_slim_ch_list,
					      dai->grph);
		taiko_codec_enable_slim_chmask(dai, false);
		wcd_slim_close_slim_sch_rx(&wcd->slim_data, &dai->wcd_slim_ch_list,
						dai->grph);
		// wcd9335_codec_vote_max_bw(codec, false);
		break;
	}
	return ret;
}

static int taiko_hph_pa_event(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);
	//enum wcd9xxx_notify_event e_pre_on, e_post_off;
	u8 req_clsh_state;
	u32 pa_settle_time = 13000; // TAIKO_HPH_PA_SETTLE_COMP_OFF;

	pr_debug("%s: %s event = %d\n", __func__, w->name, event);
	if (w->shift == 5) {
		//e_pre_on = WCD9XXX_EVENT_PRE_HPHL_PA_ON;
		//e_post_off = WCD9XXX_EVENT_POST_HPHL_PA_OFF;
		req_clsh_state = WCD_CLSH_STATE_HPHL;
	} else if (w->shift == 4) {
		//e_pre_on = WCD9XXX_EVENT_PRE_HPHR_PA_ON;
		//e_post_off = WCD9XXX_EVENT_POST_HPHR_PA_OFF;
		req_clsh_state = WCD_CLSH_STATE_HPHR;
	} else {
		pr_err("%s: Invalid w->shift %d\n", __func__, w->shift);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Let MBHC module know PA is turning on */
		// wcd9xxx_resmgr_notifier_call(&taiko->resmgr, e_pre_on);
		break;

	case SND_SOC_DAPM_POST_PMU:
		usleep_range(pa_settle_time, pa_settle_time + 1000);
		pr_debug("%s: sleep %d us after %s PA enable\n", __func__,
				pa_settle_time, w->name);
		/* wcd9xxx_clsh_fsm(codec, &taiko->clsh_d,
						 req_clsh_state,
						 WCD9XXX_CLSH_REQ_ENABLE,
						 WCD9XXX_CLSH_EVENT_POST_PA);*/

		break;

	case SND_SOC_DAPM_POST_PMD:
		usleep_range(pa_settle_time, pa_settle_time + 1000);
		pr_debug("%s: sleep %d us after %s PA disable\n", __func__,
				pa_settle_time, w->name);

		/* Let MBHC module know PA turned off */
		// wcd9xxx_resmgr_notifier_call(&taiko->resmgr, e_post_off);

		/* wcd9xxx_clsh_fsm(codec, &taiko->clsh_d,
						 req_clsh_state,
						 WCD9XXX_CLSH_REQ_DISABLE,
						 WCD9XXX_CLSH_EVENT_POST_PA); */

		break;
	}
	return 0;
}

static int taiko_hphl_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TAIKO_A_CDC_CLK_RDAC_CLK_EN_CTL,
							0x02, 0x02);
		wcd_clsh_fsm(codec, &wcd->clsh_d,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_HPHR,
			     0); /*((hph_mode == CLS_H_LOHIFI) ?
			       CLS_H_HIFI : hph_mode)); */
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TAIKO_A_CDC_CLK_RDAC_CLK_EN_CTL,
							0x02, 0x00);
	}
	return 0;
}

static int taiko_hphr_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TAIKO_A_CDC_CLK_RDAC_CLK_EN_CTL,
							0x04, 0x04);
		snd_soc_update_bits(codec, w->reg, 0x40, 0x40);

		wcd_clsh_fsm(codec, &wcd->clsh_d,
			     WCD_CLSH_EVENT_PRE_DAC,
			     WCD_CLSH_STATE_HPHR,
			     0); /*((hph_mode == CLS_H_LOHIFI) ?
			       CLS_H_HIFI : hph_mode)); */
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TAIKO_A_CDC_CLK_RDAC_CLK_EN_CTL,
							0x04, 0x00);
		snd_soc_update_bits(codec, w->reg, 0x40, 0x00);
		break;
	}
	return 0;
}

static const struct snd_kcontrol_new hphl_switch[] = {
	SOC_DAPM_SINGLE("Switch", TAIKO_A_RX_HPH_L_DAC_CTL, 6, 1, 0)
};

static const char * const rx_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4", "RX5", "RX6", "RX7"
};

static const struct soc_enum rx_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TAIKO_A_CDC_CONN_RX1_B1_CTL, 0, 12, rx_mix1_text);
static const struct soc_enum rx2_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TAIKO_A_CDC_CONN_RX2_B1_CTL, 0, 12, rx_mix1_text);

static const struct snd_kcontrol_new rx_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx_mix1_inp1_chain_enum);
static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_chain_enum);

// TAIKO_A_CDC_RX1_VOL_CTL_B2_CTL

static const char * const class_h_dsm_text[] = {
	"ZERO", "DSM_HPHL_RX1", "DSM_SPKR_RX7"
};
static const struct soc_enum class_h_dsm_enum =
	SOC_ENUM_SINGLE(TAIKO_A_CDC_CONN_CLSH_CTL, 4, 3, class_h_dsm_text);
static const struct snd_kcontrol_new class_h_dsm_mux =
	SOC_DAPM_ENUM("CLASS_H_DSM MUX Mux", class_h_dsm_enum);

static int taiko_codec_dsm_mux_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	u8 reg_val, zoh_mux_val = 0x00;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		reg_val = snd_soc_read(codec, TAIKO_A_CDC_CONN_CLSH_CTL);

		if ((reg_val & 0x30) == 0x10)
			zoh_mux_val = 0x04;
		else if ((reg_val & 0x30) == 0x20)
			zoh_mux_val = 0x08;

		if (zoh_mux_val != 0x00)
			snd_soc_update_bits(codec,
					TAIKO_A_CDC_CONN_CLSH_CTL,
					0x0C, zoh_mux_val);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, TAIKO_A_CDC_CONN_CLSH_CTL,
							0x0C, 0x00);
		break;
	}
	return 0;
}

static int slim_rx_mux_get(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = wcd->rx_port_value;
	printk("wcd->rx_port_value = %u\n", wcd->rx_port_value);
	return 0;
}

static const char *const slim_rx_mux_text[] = {
	"ZERO", "AIF1_PB", "AIF2_PB", "AIF3_PB"
};

static int slim_rx_mux_put(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget *widget = snd_soc_dapm_kcontrol_widget(kcontrol);
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct snd_soc_dapm_update *update = NULL;
	u32 port_id = widget->shift;

	wcd->rx_port_value = ucontrol->value.enumerated.item[0];
	/* value need to match the Virtual port and AIF number
	 */

	printk("slim_rx_mux_put %u %u\n", port_id, wcd->rx_port_value);

	switch (wcd->rx_port_value) {
	case 0:
		list_del_init(&wcd->slim_data.rx_chs[port_id].list);
		break;
	case 1:
		if (wcd_slim_rx_vport_validation(port_id +
			TAIKO_RX_PORT_START_NUMBER,
			&wcd->dai[AIF1_PB].wcd_slim_ch_list)) {
			goto rtn;
		}
		list_add_tail(&wcd->slim_data.rx_chs[port_id].list,
			      &wcd->dai[AIF1_PB].wcd_slim_ch_list);
		break;
	default:
		dev_err(wcd->dev, "Unknown AIF %d\n", wcd->rx_port_value);
		goto err;
	}
rtn:
	snd_soc_dapm_mux_update_power(widget->dapm, kcontrol,
					wcd->rx_port_value, e, update);

	return 0;
err:
	return -EINVAL;
}

static const struct soc_enum slim_rx_mux_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(slim_rx_mux_text), slim_rx_mux_text);

static const struct snd_kcontrol_new slim_rx_mux[TAIKO_RX_MAX] = {
	SOC_DAPM_ENUM_EXT("SLIM RX1 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
	SOC_DAPM_ENUM_EXT("SLIM RX2 Mux", slim_rx_mux_enum,
			  slim_rx_mux_get, slim_rx_mux_put),
};

static int taiko_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		wcd->rx_bias_count++;
		if (wcd->rx_bias_count == 1) {
			snd_soc_update_bits(codec, WCD9XXX_A_RX_COM_BIAS,
					    0x80, 0x80);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		wcd->rx_bias_count--;
		if (!wcd->rx_bias_count)
			snd_soc_update_bits(codec, WCD9XXX_A_RX_COM_BIAS,
					    0x80, 0x00);
		break;
	};

	return 0;
}

static const struct snd_soc_dapm_widget taiko_dapm_widgets[] = {
	/*RX stuff */
	SND_SOC_DAPM_AIF_IN_E("AIF1 PB", "AIF1 Playback", 0, SND_SOC_NOPM,
				AIF1_PB, 0, taiko_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("SLIM RX1 MUX", SND_SOC_NOPM, TAIKO_RX1, 0,
				&slim_rx_mux[TAIKO_RX1]),
	SND_SOC_DAPM_MUX("SLIM RX2 MUX", SND_SOC_NOPM, TAIKO_RX2, 0,
				&slim_rx_mux[TAIKO_RX2]),

	SND_SOC_DAPM_MIXER("SLIM RX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("SLIM RX2", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Headphone */
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_PGA_E("HPHL", TAIKO_A_RX_HPH_CNP_EN, 5, 0, NULL, 0,
		taiko_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER_E("HPHL DAC", TAIKO_A_RX_HPH_L_DAC_CTL, 7, 0,
		hphl_switch, ARRAY_SIZE(hphl_switch), taiko_hphl_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_PGA_E("HPHR", TAIKO_A_RX_HPH_CNP_EN, 4, 0, NULL, 0,
		taiko_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU |	SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("HPHR DAC", NULL, TAIKO_A_RX_HPH_R_DAC_CTL, 7, 0,
		taiko_hphr_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

		// SND_SOC_NOPM = -1 = no register?
	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("RX1 CHAIN", TAIKO_A_CDC_RX1_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 CHAIN", TAIKO_A_CDC_RX2_B6_CTL, 5, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp1_mux),

	SND_SOC_DAPM_MUX_E("CLASS_H_DSM MUX", SND_SOC_NOPM, 0, 0,
		&class_h_dsm_mux, taiko_codec_dsm_mux_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
		taiko_codec_enable_rx_bias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_SUPPLY("CDC_I2S_RX_CONN", WCD9XXX_A_CDC_CLK_OTHR_CTL, 5, 0,
			    NULL, 0),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Headset (RX MIX1 and RX MIX2) */
	{"HEADPHONE", NULL, "HPHL"},
	{"HEADPHONE", NULL, "HPHR"},

	{"HPHL DAC", NULL, "RX_BIAS"},
	{"HPHR DAC", NULL, "RX_BIAS"},

	{"HPHL DAC", "Switch", "CLASS_H_DSM MUX"},
	{"HPHR DAC", NULL, "RX2 CHAIN"},

	{"CLASS_H_DSM MUX", "DSM_HPHL_RX1", "RX1 CHAIN"},

	{"RX1 CHAIN", NULL, "RX1 MIX1"},
	{"RX2 CHAIN", NULL, "RX2 MIX1"},

	{"RX1 MIX1", NULL, "RX1 MIX1 INP1"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP1"},

	{"SLIM RX1 MUX", "AIF1_PB", "AIF1 PB"},
	{"SLIM RX2 MUX", "AIF1_PB", "AIF1 PB"},

	{"SLIM RX1", NULL, "SLIM RX1 MUX"},
	{"SLIM RX2", NULL, "SLIM RX2 MUX"},

	{"RX1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX2 MIX1 INP1", "RX2", "SLIM RX2"},
};

static int taiko_codec_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);
	int i;

	printk("taiko_codec_probe\n");

	snd_soc_codec_init_regmap(codec, wcd->regmap);
	//wcd->clsh_d.codec_version = wcd->version;
	/* Class-H Init*/
	wcd_clsh_init(&wcd->clsh_d);
	/* Default HPH Mode to Class-H HiFi */
	wcd->hph_mode = CLS_H_HIFI;
	//wcd->codec = codec;

	/* if (wcd->mclk_rate == wcd9335_MCLK_CLK_12P288MHZ)
		snd_soc_update_bits(codec, WCD9335_CODEC_RPM_CLK_MCLK_CFG,
				    0x03, 0x00);
	else if (wcd->mclk_rate == wcd9335_MCLK_CLK_9P6MHZ) */
	//	snd_soc_update_bits(codec, WCD9335_CODEC_RPM_CLK_MCLK_CFG,
				    // 0x03, 0x01);

	//set initial registers
	//wcd9335_codec_init(codec);

	for (i = 0; i < NUM_CODEC_DAIS; i++) {
		INIT_LIST_HEAD(&wcd->dai[i].wcd_slim_ch_list);
		init_waitqueue_head(&wcd->dai[i].dai_wait);
	}

#if 0
	// taiko_set_rxsb_port_format
	snd_soc_update_bits(codec, TAIKO_A_CDC_CONN_RX_SB_B1_CTL, 0xff, 0x02);
	snd_soc_update_bits(codec, TAIKO_A_CDC_CONN_RX_SB_B2_CTL, 0xff, 0x0a);

	snd_soc_write(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0xD4);
	snd_soc_write(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0xD5);
	snd_soc_write(codec, WCD9XXX_A_BIAS_CENTRAL_BG_CTL, 0x55);

	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN1, 4);
	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN1, 5);
	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN2, 0);
	snd_soc_write(codec, WCD9XXX_A_CLK_BUFF_EN2, 4);

	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B1_CTL, 0x44);
	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B6_CTL, 0x2f);
	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B2_CTL, 0x03);
	snd_soc_write(codec, WCD9XXX_A_CDC_MBHC_TIMER_B3_CTL, 0x17);

	snd_soc_write(codec, TAIKO_A_RX_COM_BIAS, 0x80);

	snd_soc_write(codec, TAIKO_A_CDC_CONN_CLSH_CTL, 0x14);
	snd_soc_write(codec, TAIKO_A_CDC_CLK_RDAC_CLK_EN_CTL, 0x04);
	snd_soc_write(codec, WCD9XXX_A_RX_HPH_R_DAC_CTL, 0x40);
	snd_soc_write(codec, WCD9XXX_A_BUCK_CTRL_CCL_4, 0x50);
	snd_soc_write(codec, TAIKO_A_BUCK_CTRL_CCL_1, 0x5b);

	snd_soc_write(codec, TAIKO_A_BUCK_CTRL_CCL_3, 0x68);
	snd_soc_write(codec, TAIKO_A_BUCK_CTRL_CCL_3, 0x60);

	snd_soc_write(codec, TAIKO_A_CDC_CLSH_BUCK_NCP_VARS, 0x04);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B2_CTL, 0x01);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B2_CTL, 0x05);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B2_CTL, 0x35);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B3_CTL, 0x30);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B3_CTL, 0x3b);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B1_CTL, 0xe6);
	snd_soc_write(codec, TAIKO_A_CDC_CLSH_B1_CTL, 0xa6);

	//snd_soc_write(codec, TAIKO_A_CDC_CLSH_V_PA_HD_HPH, 0x0d);
	//snd_soc_write(codec, TAIKO_A_CDC_CLSH_V_PA_MIN_HPH, 0x1d);
	//snd_soc_write(codec, TAIKO_A_CDC_CLSH_IDLE_HPH_THSD, 0x13);
snd_soc_write(codec, 0xB2F - 0x800, 0xD);
snd_soc_write(codec, 0xB31 - 0x800, 0x1D);
snd_soc_write(codec, 0xB24 - 0x800, 0x13);
snd_soc_write(codec, 0xB26 - 0x800, 0x19);
snd_soc_write(codec, 0xB2A - 0x800, 0x97);
snd_soc_write(codec, 0xB29 - 0x800, 0xAE);
snd_soc_write(codec, 0xB29 - 0x800, 0x1);
snd_soc_write(codec, 0xB29 - 0x800, 0x1C);
snd_soc_write(codec, 0xB29 - 0x800, 0x0);
snd_soc_write(codec, 0xB29 - 0x800, 0x24);
snd_soc_write(codec, 0xB29 - 0x800, 0x0);
snd_soc_write(codec, 0xB29 - 0x800, 0x25);
snd_soc_write(codec, 0xB29 - 0x800, 0x0);
snd_soc_write(codec, 0xB20 - 0x800, 0xA7);
snd_soc_write(codec, 0xB0C - 0x800, 0x1);
snd_soc_write(codec, 0xB20 - 0x800, 0xA3);
snd_soc_write(codec, 0x985 - 0x800, 0x2);
snd_soc_write(codec, 0x984 - 0x800, 0xFF);
snd_soc_write(codec, 0x981 - 0x800, 0x25);
snd_soc_write(codec, 0x983 - 0x800, 0xCA);
snd_soc_write(codec, 0x983 - 0x800, 0xC2);
snd_soc_write(codec, 0x981 - 0x800, 0xA5);
snd_soc_write(codec, 0x992 - 0x800, 0xFF);
snd_soc_write(codec, 0x9B7 - 0x800, 0xC0);
snd_soc_write(codec, 0xB0D - 0x800, 0x6);
snd_soc_write(codec, 0x9B1 - 0x800, 0xC0);
snd_soc_write(codec, 0xAB5 - 0x800, 0xA0);
snd_soc_write(codec, 0xABD - 0x800, 0xA0);
snd_soc_write(codec, 0xB01 - 0x800, 0x1);
snd_soc_write(codec, 0xB01 - 0x800, 0x0);
snd_soc_write(codec, 0xB01 - 0x800, 0x2);
snd_soc_write(codec, 0xB01 - 0x800, 0x0);
snd_soc_write(codec, 0xB0F - 0x800, 0x3);
snd_soc_write(codec, 0xAB7 - 0x800, 0x0);
snd_soc_write(codec, 0xABF - 0x800, 0x0);
snd_soc_write(codec, 0x980 - 0x800, 0x3);
snd_soc_write(codec, 0x840 - 0x800, 0x5);
snd_soc_write(codec, 0x984 - 0x800, 0x3);
snd_soc_write(codec, 0x841 - 0x800, 0x5);
snd_soc_write(codec, 0x9AB - 0x800, 0xB0);
snd_soc_write(codec, 0x985 - 0x800, 0x0);
snd_soc_write(codec, 0x994 - 0x800, 0x8);
snd_soc_write(codec, 0x983 - 0x800, 0xC6);
snd_soc_write(codec, 0x983 - 0x800, 0xCE);
#endif

	return 0; //wcd9335_setup_irqs(wcd);
}

static int taiko_codec_remove(struct snd_soc_codec *codec)
{
	//struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);

	//wcd9335_cleanup_irqs(wcd);

	return 0;
}

static int taiko_codec_set_sysclk(struct snd_soc_codec *codec,
				    int clk_id, int source,
				    unsigned int freq, int dir)
{
	//struct wcd9335 *wcd = snd_soc_codec_get_drvdata(codec);

	return 0;
}

static struct snd_soc_codec_driver taiko_codec_drv = {
	.probe = taiko_codec_probe,
	.remove = taiko_codec_remove,
	.set_sysclk = taiko_codec_set_sysclk,
	.component_driver = {
		.controls = 0, //taiko_snd_controls,
		.num_controls = 0, //ARRAY_SIZE(taiko_snd_controls),
		.dapm_widgets = taiko_dapm_widgets,
		.num_dapm_widgets = ARRAY_SIZE(taiko_dapm_widgets),
		.dapm_routes = audio_map,
		.num_dapm_routes = ARRAY_SIZE(audio_map),
	}
};

static int wcd9335_slim_get_laddr(struct slim_device *sb,
				  const u8 *e_addr, u8 e_len, u8 *laddr)
{
	int ret;
	const unsigned long timeout = jiffies +
				      msecs_to_jiffies(SLIMBUS_PRESENT_TIMEOUT);

	do {
		ret = slim_get_logical_addr(sb, e_addr, e_len, laddr);
		if (!ret)
			break;
		/* Give SLIMBUS time to report present and be ready. */
		usleep_range(1000, 1100);
		pr_debug_ratelimited("%s: retyring get logical addr\n",
				     __func__);
	} while time_before(jiffies, timeout);

	return ret;
}

static int wcd9335_parse_slim_ifd_info(struct device *dev, struct slim_device *slim_ifd)
{
	int ret = 0;
	struct property *prop;

	ret = of_property_read_string(dev->of_node, "qcom,cdc-slim-ifd",
				      &slim_ifd->name);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"qcom,cdc-slim-ifd-dev", dev->of_node->full_name);
		return -ENODEV;
	}
	prop = of_find_property(dev->of_node,
			"qcom,cdc-slim-ifd-elemental-addr", NULL);
	if (!prop) {
		dev_err(dev, "Looking up %s property in node %s failed",
			"qcom,cdc-slim-ifd-elemental-addr",
			dev->of_node->full_name);
		return -ENODEV;
	} else if (prop->length != 6) {
		dev_err(dev, "invalid codec slim ifd addr. addr length = %d\n",
			      prop->length);
		return -ENODEV;
	}
	memcpy(slim_ifd->e_addr, prop->value, 6);
	return 0;
}

static int wcd9335_parse_dt(struct wcd9335 *wcd)
{
	struct device *dev = wcd->dev;
	struct device_node *np = dev->of_node;
	int ret;

	wcd->reset_gpio = of_get_named_gpio(np,	"qcom,cdc-reset-gpio", 0);
	if (wcd->reset_gpio < 0) {
		dev_err(dev, "Reset gpio missing in DT\n");
		return -EINVAL;
	}

	wcd->irq_gpio = of_get_named_gpio(np, "qcom,gpio-int2", 0);
	if (!gpio_is_valid(wcd->irq_gpio)) {
		dev_err(dev, "IRQ gpio missing in DT\n");
		return -EINVAL;
	}

	wcd->clk1_gpio = of_get_named_gpio(np, "qcom,clk1-gpio", 0);
	if (!gpio_is_valid(wcd->clk1_gpio)) {
		dev_err(dev, "CLK gpio missing in DT\n");
		return -EINVAL;
	}

	gpio_request(wcd->clk1_gpio, "CLK1");
	gpio_direction_output(wcd->clk1_gpio, 1);

	//FIXME should go in to machine driver
	ret = of_property_read_u32(np, "qcom,cdc-mclk-clk-rate", &wcd->mclk_rate);
	if (ret) {
		dev_err(dev, "Reset mclk rate missing in DT\n");
		return -EINVAL;
	}

	if (wcd->mclk_rate != WCD9XXX_MCLK_CLK_9P6HZ &&
	    wcd->mclk_rate != WCD9XXX_MCLK_CLK_12P288MHZ) {
		dev_err(dev, "Invalid mclk_rate = %u\n", wcd->mclk_rate);
		return -EINVAL;
	}

	wcd->ext_clk = devm_clk_get(dev, "mclk");
	if (IS_ERR(wcd->ext_clk)) {
		dev_err(dev, "Unable to find external clk\n");
		return -EINVAL;
	}

	clk_set_rate(wcd->ext_clk, 9600000);

	clk_prepare_enable(wcd->ext_clk);

	wcd->native_clk = NULL; /*devm_clk_get(dev, "native");
	if (IS_ERR(wcd->native_clk)) {
		dev_err(dev, "Unable to find native clk\n");
		return -EINVAL;
	}*/

	return 0;
}

void wcd9335_reset(struct wcd9335 *wcd)
{
	gpio_direction_output(wcd->reset_gpio, 0);
	msleep(20);
	gpio_set_value(wcd->reset_gpio, 1);
	msleep(20);
}

int wcd9335_power_up(struct wcd9335 *wcd)
{
	struct device *dev = wcd->dev;
	int ret;

	wcd->num_of_supplies = 7;
	wcd->supplies[0].supply = "vdd-buck";
	//wcd->supplies[1].supply = "buck-sido";
	wcd->supplies[1].supply = "vdd-tx-h";
	wcd->supplies[2].supply = "vdd-rx-h";
	wcd->supplies[3].supply = "vddpx-1";

	wcd->supplies[4].supply = "vdd-a-1p2v";
	wcd->supplies[5].supply = "vddcx-1";
	wcd->supplies[6].supply = "vddcx-2";

	ret = regulator_bulk_get(dev, wcd->num_of_supplies, wcd->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	ret = regulator_bulk_enable(wcd->num_of_supplies, wcd->supplies);
	if (ret != 0) {
		dev_err(dev, "Failed to get supplies: err = %d\n", ret);
		return ret;
	}

	/*
	 * For WCD9335, it takes about 600us for the Vout_A and
	 * Vout_D to be ready after BUCK_SIDO is powered up.
	 * SYS_RST_N shouldn't be pulled high during this time
	 */
	usleep_range(600, 650);


	return 0;
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct wcd9335 *wcd = dev_id;

	unsigned status[3] = {0}, istatus[4] = {0}, tmp = 0;
	unsigned i;

	regmap_read(wcd->regmap, WCD9XXX_A_INTR_STATUS0 + 0, &status[0]);
	regmap_read(wcd->regmap, WCD9XXX_A_INTR_STATUS0 + 1, &status[1]);
	regmap_read(wcd->regmap, WCD9XXX_A_INTR_STATUS0 + 2, &status[2]);

	regmap_write(wcd->regmap, WCD9XXX_A_INTR_CLEAR0 + 0, status[0]);
	regmap_write(wcd->regmap, WCD9XXX_A_INTR_CLEAR0 + 1, status[1]);
	regmap_write(wcd->regmap, WCD9XXX_A_INTR_CLEAR0 + 2, status[2]);

	if (status[0] & 1) {
	regmap_read(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_STATUS_RX_0 + 0, &istatus[0]);
	regmap_read(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_STATUS_RX_0 + 1, &istatus[1]);
	regmap_read(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_STATUS_RX_0 + 2, &istatus[2]);
	regmap_read(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_STATUS_RX_0 + 3, &istatus[3]);
	printk("irq2 %X %X %X %X\n", istatus[0], istatus[1], istatus[2], istatus[3]);
	regmap_write(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_CLR_RX_0 + 0, istatus[0]);
	regmap_write(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_CLR_RX_0 + 1, istatus[1]);
	regmap_write(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_CLR_RX_0 + 2, istatus[2]);
	regmap_write(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_CLR_RX_0 + 3, istatus[3]);

	for (i = 0; i < 32; i++) {
		if (istatus[i / 8] & (1 << i & 7)) {
			regmap_read(wcd->ifd_regmap, TAIKO_SLIM_PGD_PORT_INT_RX_SOURCE0 + i, &tmp);
			printk("%u %u\n", i, tmp);
		}
	}

	}

	printk("irq %i %X %X %X\n", irq, status[0], status[1], status[2]);

	return IRQ_HANDLED;
}

static int wcd9335_init(struct wcd9335 *wcd)
{
	int ret;

	wcd->irq = gpio_to_irq(wcd->irq_gpio);
	if (wcd->irq < 0) {
		pr_err("Unable to configure irq\n");
		return wcd->irq;
	}

	ret = devm_request_threaded_irq(wcd->dev, wcd->irq, NULL,
					irq_handler,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"wcd", wcd);

	/*ret = regmap_add_irq_chip(wcd->regmap, wcd->irq,
			   	 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				 0, &wcd9335_regmap_irq1_chip,
				 &wcd->irq_data);
	if (ret != 0) {
       	        pr_err("Failed to register IRQ chip: %d\n", ret);
               	return ret;
       	} */

	return 0;
}

struct reg_mask_val {
	u16 reg;
	u8 mask;
	u8 val;
};

static const struct reg_mask_val reg_init_val[] = {
	/* Initialize current threshold to 350MA
	 * number of wait and run cycles to 4096
	 */
	{TAIKO_A_RX_HPH_OCP_CTL, 0xE1, 0x61},
	{TAIKO_A_RX_COM_OCP_COUNT, 0xFF, 0xFF},
	{TAIKO_A_RX_HPH_L_TEST, 0x01, 0x01},
	{TAIKO_A_RX_HPH_R_TEST, 0x01, 0x01},

	/* Initialize gain registers to use register gain */
	{TAIKO_A_RX_HPH_L_GAIN, 0x20, 0x20},
	{TAIKO_A_RX_HPH_R_GAIN, 0x20, 0x20},
	{TAIKO_A_RX_LINE_1_GAIN, 0x20, 0x20},
	{TAIKO_A_RX_LINE_2_GAIN, 0x20, 0x20},
	{TAIKO_A_RX_LINE_3_GAIN, 0x20, 0x20},
	{TAIKO_A_RX_LINE_4_GAIN, 0x20, 0x20},
	{TAIKO_A_SPKR_DRV_GAIN, 0x04, 0x04},

	/* Use 16 bit sample size for TX1 to TX6 */
	{TAIKO_A_CDC_CONN_TX_SB_B1_CTL, 0x30, 0x20},
	{TAIKO_A_CDC_CONN_TX_SB_B2_CTL, 0x30, 0x20},
	{TAIKO_A_CDC_CONN_TX_SB_B3_CTL, 0x30, 0x20},
	{TAIKO_A_CDC_CONN_TX_SB_B4_CTL, 0x30, 0x20},
	{TAIKO_A_CDC_CONN_TX_SB_B5_CTL, 0x30, 0x20},
	{TAIKO_A_CDC_CONN_TX_SB_B6_CTL, 0x30, 0x20},

	/* Use 16 bit sample size for TX7 to TX10 */
	{TAIKO_A_CDC_CONN_TX_SB_B7_CTL, 0x60, 0x40},
	{TAIKO_A_CDC_CONN_TX_SB_B8_CTL, 0x60, 0x40},
	{TAIKO_A_CDC_CONN_TX_SB_B9_CTL, 0x60, 0x40},
	{TAIKO_A_CDC_CONN_TX_SB_B10_CTL, 0x60, 0x40},

	/*enable HPF filter for TX paths */
	{TAIKO_A_CDC_TX1_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX2_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX3_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX4_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX5_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX6_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX7_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX8_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX9_MUX_CTL, 0x8, 0x0},
	{TAIKO_A_CDC_TX10_MUX_CTL, 0x8, 0x0},

	/* Compander zone selection */
	{TAIKO_A_CDC_COMP0_B4_CTL, 0x3F, 0x37},
	{TAIKO_A_CDC_COMP1_B4_CTL, 0x3F, 0x37},
	{TAIKO_A_CDC_COMP2_B4_CTL, 0x3F, 0x37},
	{TAIKO_A_CDC_COMP0_B5_CTL, 0x7F, 0x7F},
	{TAIKO_A_CDC_COMP1_B5_CTL, 0x7F, 0x7F},
	{TAIKO_A_CDC_COMP2_B5_CTL, 0x7F, 0x7F},

	/*
	 * Setup wavegen timer to 20msec and disable chopper
	 * as default. This corresponds to Compander OFF
	 */
	{TAIKO_A_RX_HPH_CNP_WG_CTL, 0xFF, 0xDB},
	{TAIKO_A_RX_HPH_CNP_WG_TIME, 0xFF, 0x58},
	{TAIKO_A_RX_HPH_BIAS_WG_OCP, 0xFF, 0x1A},
	{TAIKO_A_RX_HPH_CHOP_CTL, 0xFF, 0x24},

	/* Choose max non-overlap time for NCP */
	{TAIKO_A_NCP_CLK, 0xFF, 0xFC},

	/* Program the 0.85 volt VBG_REFERENCE */
	{TAIKO_A_BIAS_CURR_CTL_2, 0xFF, 0x04},

	/* set MAD input MIC to DMIC1 */
	// {TAIKO_A_CDC_CONN_MAD, 0x0F, 0x08},
};

#include "wcd9320-tables.c"

static int wcd9335_bring_up(struct wcd9335 *wcd)
{
	//int val;
	int ret = 0;
	unsigned i;
	unsigned bias_msb, bias_lsb;
	short bias_value;

	//for (i = 0; i < ARRAY_SIZE(taiko_reset_reg_defaults); i++)
	//	reg_write(wcd->slim, i, taiko_reset_reg_defaults[i]);

	reg_write(wcd->slim, WCD9XXX_A_LEAKAGE_CTL, 0x4);
	reg_write(wcd->slim, WCD9XXX_A_CDC_CTL, 0);
	usleep_range(5000, 5000);
	reg_write(wcd->slim, WCD9XXX_A_CDC_CTL, 3);
	reg_write(wcd->slim, WCD9XXX_A_LEAKAGE_CTL, 3);

	// irq init
	reg_write(wcd->slim, WCD9XXX_A_INTR_LEVEL0 + 0, 1);
	reg_write(wcd->slim, WCD9XXX_A_INTR_MASK0 + 0 , 0xff);

	reg_write(wcd->slim, WCD9XXX_A_INTR_LEVEL0 + 1, 0);
	reg_write(wcd->slim, WCD9XXX_A_INTR_MASK0 + 1 , 0xff);

	reg_write(wcd->slim, WCD9XXX_A_INTR_LEVEL0 + 2, 0);
	reg_write(wcd->slim, WCD9XXX_A_INTR_MASK0 + 2 , 0xff);

	reg_write(wcd->slim, WCD9XXX_A_INTR_LEVEL0 + 3, 0);
	reg_write(wcd->slim, WCD9XXX_A_INTR_MASK0 + 3 , 0x7f);

	// stuff happens...
	reg_write(wcd->slim, WCD9XXX_A_INTR_MASK0 + 2 , 0xff);

	// init base
	reg_write(wcd->slim, TAIKO_A_CHIP_CTL, 2);
	reg_write(wcd->slim, TAIKO_A_CDC_CLK_POWER_CTL, 3);

	reg_write(wcd->slim, TAIKO_A_RX_EAR_CMBUFF, 5);

	reg_write(wcd->slim, TAIKO_A_CDC_RX1_B5_CTL, 0x78);
	reg_write(wcd->slim, TAIKO_A_CDC_RX2_B5_CTL, 0x78);
	reg_write(wcd->slim, TAIKO_A_CDC_RX3_B5_CTL, 0x78);
	reg_write(wcd->slim, TAIKO_A_CDC_RX4_B5_CTL, 0x78);
	reg_write(wcd->slim, TAIKO_A_CDC_RX5_B5_CTL, 0x78);
	reg_write(wcd->slim, TAIKO_A_CDC_RX6_B5_CTL, 0x78);
	reg_write(wcd->slim, TAIKO_A_CDC_RX7_B5_CTL, 0x78);

	reg_write(wcd->slim, TAIKO_A_CDC_RX1_B6_CTL, 0xA0);
	reg_write(wcd->slim, TAIKO_A_CDC_RX2_B6_CTL, 0xA0);

	reg_write(wcd->slim, TAIKO_A_CDC_RX3_B6_CTL, 0x80);
	reg_write(wcd->slim, TAIKO_A_CDC_RX4_B6_CTL, 0x80);
	reg_write(wcd->slim, TAIKO_A_CDC_RX5_B6_CTL, 0x80);
	reg_write(wcd->slim, TAIKO_A_CDC_RX6_B6_CTL, 0x80);
	reg_write(wcd->slim, TAIKO_A_CDC_RX7_B6_CTL, 0x80);

	reg_write(wcd->slim, TAIKO_A_MAD_ANA_CTRL, 0xF1);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_MAIN_CTL_1, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_MAIN_CTL_2, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_1, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_2, 0x03); //
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_3, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_4, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_5, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_6, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_7, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_CTL_8, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_IIR_CTL_PTR, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_MAD_AUDIO_IIR_CTL_VAL, 0x40);
	reg_write(wcd->slim, TAIKO_A_CDC_DEBUG_B7_CTL, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_CLK_OTHR_RESET_B1_CTL, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_CLK_OTHR_CTL, 0x00);
	reg_write(wcd->slim, TAIKO_A_CDC_CONN_MAD, 0x01);

	// init taiko v2
#define TAIKO_REG_VAL(x, y) regmap_write(wcd->regmap, x, y)
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_1_GAIN, 0x2),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_2_GAIN, 0x2),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_1_2_ADC_IB, 0x44),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_3_GAIN, 0x2),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_4_GAIN, 0x2),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_3_4_ADC_IB, 0x44),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_5_GAIN, 0x2),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_6_GAIN, 0x2),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX_5_6_ADC_IB, 0x44),
	TAIKO_REG_VAL(WCD9XXX_A_BUCK_MODE_3, 0xCE),
	TAIKO_REG_VAL(WCD9XXX_A_BUCK_CTRL_VCL_1, 0x8),
	TAIKO_REG_VAL(WCD9XXX_A_BUCK_CTRL_CCL_4, 0x51),
	TAIKO_REG_VAL(TAIKO_A_NCP_DTEST, 0x10),
	TAIKO_REG_VAL(TAIKO_A_RX_HPH_CHOP_CTL, 0xA4),
	TAIKO_REG_VAL(TAIKO_A_RX_HPH_BIAS_PA, 0x7A),
	TAIKO_REG_VAL(TAIKO_A_RX_HPH_OCP_CTL, 0x69),
	TAIKO_REG_VAL(TAIKO_A_RX_HPH_CNP_WG_CTL, 0xDA),
	TAIKO_REG_VAL(TAIKO_A_RX_HPH_CNP_WG_TIME, 0x15),
	TAIKO_REG_VAL(TAIKO_A_RX_EAR_BIAS_PA, 0x76),
	TAIKO_REG_VAL(TAIKO_A_RX_EAR_CNP, 0xC0),
	TAIKO_REG_VAL(TAIKO_A_RX_LINE_BIAS_PA, 0x78),
	TAIKO_REG_VAL(TAIKO_A_RX_LINE_1_TEST, 0x2),
	TAIKO_REG_VAL(TAIKO_A_RX_LINE_2_TEST, 0x2),
	TAIKO_REG_VAL(TAIKO_A_RX_LINE_3_TEST, 0x2),
	TAIKO_REG_VAL(TAIKO_A_RX_LINE_4_TEST, 0x2),
	TAIKO_REG_VAL(TAIKO_A_SPKR_DRV_OCP_CTL, 0x97),
	TAIKO_REG_VAL(TAIKO_A_SPKR_DRV_CLIP_DET, 0x1),
	TAIKO_REG_VAL(TAIKO_A_SPKR_DRV_IEC, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX1_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX2_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX3_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX4_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX5_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX6_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX7_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX8_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX9_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_TX10_MUX_CTL, 0x48),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX1_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX2_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX3_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX4_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX5_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX6_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_RX7_B4_CTL, 0x8),
	TAIKO_REG_VAL(TAIKO_A_CDC_VBAT_GAIN_UPD_MON, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_PA_RAMP_B1_CTL, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_PA_RAMP_B2_CTL, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_PA_RAMP_B3_CTL, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_PA_RAMP_B4_CTL, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_SPKR_CLIPDET_B1_CTL, 0x0),
	TAIKO_REG_VAL(TAIKO_A_CDC_COMP0_B4_CTL, 0x37),
	TAIKO_REG_VAL(TAIKO_A_CDC_COMP0_B5_CTL, 0x7f);

	// other init
	for (i = 0; i < ARRAY_SIZE(reg_init_val); i++) {
		regmap_update_bits(wcd->regmap, reg_init_val[i].reg, reg_init_val[i].mask, reg_init_val[i].val);
	};

	regmap_update_bits(wcd->regmap, TAIKO_A_LDO_H_MODE_1, 0x0C, 0xc);
			    //(pdata->micbias.ldoh_v << 2));

	regmap_update_bits(wcd->regmap, TAIKO_A_MICB_CFILT_1_VAL, 0xFC, 0x60); //(k1 << 2));
	regmap_update_bits(wcd->regmap, TAIKO_A_MICB_CFILT_2_VAL, 0xFC, 0x9c); //(k2 << 2));
	regmap_update_bits(wcd->regmap, TAIKO_A_MICB_CFILT_3_VAL, 0xFC, 0x60); //(k3 << 2));

	//regmap_update_bits(wcd->regmap, TAIKO_A_MICB_1_CTL, 0x60,
	//		    (pdata->micbias.bias1_cfilt_sel << 5));
	regmap_update_bits(wcd->regmap, TAIKO_A_MICB_2_CTL, 0x60, 0x20);
			    //(pdata->micbias.bias2_cfilt_sel << 5));
	//regmap_update_bits(wcd->regmap, TAIKO_A_MICB_3_CTL, 0x60,
//			    (pdata->micbias.bias3_cfilt_sel << 5));
	//regmap_update_bits(wcd->regmap, taiko->resmgr.reg_addr->micb_4_ctl, 0x60,
			    //(pdata->micbias.bias4_cfilt_sel << 5));

	regmap_write(wcd->regmap, TAIKO_A_CDC_ANC1_B2_CTL, 1);
	regmap_write(wcd->regmap, WCD9XXX_A_INTR_MASK0, 0xfe);

	reg_write(&wcd->slim_ifd, TAIKO_SLIM_PGD_PORT_INT_EN0 + 0, 0xff);
	reg_write(&wcd->slim_ifd, TAIKO_SLIM_PGD_PORT_INT_EN0 + 1, 0xff);
	reg_write(&wcd->slim_ifd, TAIKO_SLIM_PGD_PORT_INT_EN0 + 2, 0xff);

	regmap_write(wcd->regmap, TAIKO_A_CDC_RX1_B6_CTL, 0x80);
	regmap_write(wcd->regmap, TAIKO_A_CDC_RX2_B6_CTL, 0x80);

	regmap_write(wcd->regmap, WCD9XXX_A_MICB_CFILT_2_CTL, 0);

	regmap_write(wcd->regmap, TAIKO_A_BIAS_CENTRAL_BG_CTL, 0xd0);
	regmap_write(wcd->regmap, TAIKO_A_BIAS_CENTRAL_BG_CTL, 0xd4);
	regmap_write(wcd->regmap, TAIKO_A_BIAS_CENTRAL_BG_CTL, 0xd5);
	regmap_write(wcd->regmap, TAIKO_A_BIAS_CENTRAL_BG_CTL, 0x55);

	regmap_write(wcd->regmap, TAIKO_A_CLK_BUFF_EN1, 5);
	regmap_write(wcd->regmap, TAIKO_A_CLK_BUFF_EN2, 0);
	regmap_write(wcd->regmap, TAIKO_A_CLK_BUFF_EN2, 4);
	regmap_write(wcd->regmap, TAIKO_A_CDC_CLK_MCLK_CTL, 1);

	// taiko_hs_detect

	// mbhc clock

	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_TIMER_B1_CTL, 0x44);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_TIMER_B6_CTL, 0x2f);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_TIMER_B2_CTL, 0x03);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_TIMER_B3_CTL, 0x17);

	// mhbc setup

	for (i = 0; i < 8; i++) {
		regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_FIR_B1_CFG, 0x07, i);
		regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_FIR_B2_CFG,
			(uint8_t[8]) {0x3e, 0x7c} [i]);
	}

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B2_CTL, 0x07, 1);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_TIMER_B4_CTL, 0x70, 0x40);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_TIMER_B4_CTL, 0x0f, 3);

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_TIMER_B5_CTL, 4);

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x80, 0x80);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x78, 0x58);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x02, 0x02);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_2, 0xF0, 0xF0);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B2_CTL, 0x78, 0x48);

	/* */

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x02, 0x00);

	regmap_write(wcd->regmap, WCD9XXX_A_TX_7_MBHC_TEST_CTL, 0x78);

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x04, 0x04);

	regmap_update_bits(wcd->regmap, WCD9XXX_A_MICB_2_CTL, 1, 1);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MAD_ANA_CTRL, 0x10, 0);

	regmap_write(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x02);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x80, 0x80);

	msleep(50);

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x0a);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x02);

	// ?
	{
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x8, 0x8);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x4);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x8, 0x0);
	//usleep_range(mbhc->mbhc_data.t_sta_dce, mbhc->mbhc_data.t_sta_dce);
	usleep_range(2000, 2000);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x4);
	//usleep_range(mbhc->mbhc_data.t_dce, mbhc->mbhc_data.t_dce);
	usleep_range(10000, 10000);

	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B5_STATUS, &bias_msb);
	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B4_STATUS, &bias_lsb);
	bias_value = (bias_msb << 8) | bias_lsb;
	printk("BIAS_VALUE=%i\n", bias_value);
	}

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x0a);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x02);

	// ?
	{
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x8, 0x8);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x2);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x8, 0x0);
	//usleep_range(mbhc->mbhc_data.t_sta_dce, mbhc->mbhc_data.t_sta_dce);
	usleep_range(2000, 2000);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x2);
	//usleep_range(mbhc->mbhc_data.t_sta, mbhc->mbhc_data.t_sta);
	usleep_range(500, 500);

	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B3_STATUS, &bias_msb);
	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B2_STATUS, &bias_lsb);
	bias_value = (bias_msb << 8) | bias_lsb;
	printk("BIAS_VALUE=%i\n", bias_value);

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x8, 0x8);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x0);
	}

	regmap_update_bits(wcd->regmap, WCD9XXX_A_MAD_ANA_CTRL, 0x10, 0x10);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MICB_2_CTL, 1, 0);

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x0a);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x02);
	regmap_write(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x02);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x80, 0x80);

	msleep(50);

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x04);
	//usleep_range(??);
	usleep_range(10000, 10000);
	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B5_STATUS, &bias_msb);
	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B4_STATUS, &bias_lsb);
	bias_value = (bias_msb << 8) | bias_lsb;
	printk("BIAS_VALUE=%i\n", bias_value);

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x0a);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x02);
	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_CLK_CTL, 0x02);
	regmap_write(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x02);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x80, 0x80);

	msleep(50);

	regmap_write(wcd->regmap, WCD9XXX_A_CDC_MBHC_EN_CTL, 0x02);
	//usleep_range(??);
	usleep_range(2000, 2000);
	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B3_STATUS, &bias_msb);
	regmap_read(wcd->regmap, WCD9XXX_A_CDC_MBHC_B2_STATUS, &bias_lsb);
	bias_value = (bias_msb << 8) | bias_lsb;
	printk("BIAS_VALUE=%i\n", bias_value);

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x04, 0x00);
	// snd_soc_write(codec, mbhc->mbhc_bias_regs.cfilt_ctl, cfilt_mode);
	regmap_write(wcd->regmap, WCD9XXX_A_MICB_CFILT_2_CTL, 0);
	regmap_write(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x04);
	regmap_update_bits(wcd->regmap, WCD9XXX_A_MBHC_SCALING_MUX_1, 0x80, 0x80);
	usleep_range(100, 100);

	regmap_update_bits(wcd->regmap, WCD9XXX_A_CDC_MBHC_B1_CTL, 0x02, 0x02);

	/* */
	regmap_write(wcd->regmap, TAIKO_A_CLK_BUFF_EN2, 0);
	regmap_write(wcd->regmap, TAIKO_A_CLK_BUFF_EN2, 2);
	regmap_write(wcd->regmap, TAIKO_A_CLK_BUFF_EN1, 0);
	regmap_write(wcd->regmap, TAIKO_A_BIAS_CENTRAL_BG_CTL, 0x54);

	/*
	<4>[   13.685430] slim_change_val_element BD0 A8 1 ED437400
<4>[   13.686022] slim_change_val_element BD1  2 1 ED437400
<4>[   13.686505] slim_change_val_element BD2 A5 1 ED437400
<4>[   13.687096] slim_change_val_element BD3 FC 1 ED437400
<4>[   13.687502] slim_change_val_element BD4 97 1 ED437400
<4>[   13.688085] slim_change_val_element BD5 FE 1 ED437400
<4>[   13.688492] slim_change_val_element BD8 97 1 ED437400
<4>[   13.688899] slim_change_val_element BD9 FE 1 ED437400
<4>[   13.689481] slim_change_val_element BDA  0 1 ED437400
<4>[   13.689888] slim_change_val_element BDB 80 1 ED437400
<4>[   13.690534] slim_change_val_element 931 37 1 ED437400
<4>[   13.690956] slim_change_val_element 9FE 45 1 ED437400
<4>[   13.691552] slim_change_val_element 9AA 79 1 ED437400
<4>[   13.692020] slim_change_val_element 896 FE 1 ED437400
<4>[   13.692665] slim_change_val_element 896 FC 1 ED437400
<4>[   13.693094] slim_change_val_element 9AA 7B 1 ED437400
<4>[   13.694062] slim_change_val_element 897 6F 1 ED437400
<4>[   13.694674] slim_change_val_element 94A 6E 1 ED437400
<4>[   13.695098] slim_change_val_element 94A 6F 1 ED437400

	*/

	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B1_CTL, 0xa8);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B2_CTL, 0x02);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B3_CTL, 0xa5);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B4_CTL, 0xfc);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B5_CTL, 0x97);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B6_CTL, 0xfe);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B9_CTL, 0x97);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B10_CTL, 0xfe);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B11_CTL, 0x0);
	regmap_write(wcd->regmap, TAIKO_A_CDC_MBHC_VOLT_B12_CTL, 0x80);

	regmap_write(wcd->regmap, WCD9XXX_A_MICB_2_CTL, 0x37);
	regmap_write(wcd->regmap, WCD9XXX_A_MBHC_HPH, 0x45);
	regmap_write(wcd->regmap, TAIKO_A_RX_HPH_OCP_CTL, 0x79);
	regmap_write(wcd->regmap, TAIKO_A_INTR_MASK2, 0xfe);
	regmap_write(wcd->regmap, TAIKO_A_INTR_MASK2, 0xfc);
	regmap_write(wcd->regmap, TAIKO_A_RX_HPH_OCP_CTL, 0x7b);
	regmap_write(wcd->regmap, TAIKO_A_INTR_MASK3, 0x6f);
	regmap_write(wcd->regmap, TAIKO_A_MBHC_INSERT_DETECT, 0x6e);
	regmap_write(wcd->regmap, TAIKO_A_MBHC_INSERT_DETECT, 0x6f);

	/* reg_write(wcd->slim, TAIKO_A_RX_HPH_L_GAIN, 0x20);
	reg_write(wcd->slim, TAIKO_A_RX_HPH_R_GAIN, 0x20);

	printk("%u %u %u %u %u\n",
		reg_read(wcd->slim, TAIKO_A_RX_HPH_L_GAIN),
		reg_read(wcd->slim, TAIKO_A_RX_HPH_R_GAIN),
		reg_read(wcd->slim, TAIKO_A_RX_HPH_L_DAC_CTL),
		reg_read(wcd->slim, TAIKO_A_RX_HPH_L_STATUS),
		reg_read(wcd->slim, TAIKO_A_RX_HPH_R_STATUS)
		);

	reg_write(wcd->slim, TAIKO_A_RX_HPH_L_GAIN, 20);
	reg_write(wcd->slim, TAIKO_A_RX_HPH_R_GAIN, 20);
	reg_write(wcd->slim, TAIKO_A_RX_HPH_L_DAC_CTL, 1 << 6); */

	/*for (i = 0; i < 0x100; i++) {
		printk("reg %X: %X\n", i, reg_read(wcd->slim, i));
	} */

	/* printk("OSC_FREQ=%X, %X %X %X\n",
		reg_read(wcd->slim, WCD9XXX_A_RC_OSC_FREQ),
		reg_read(wcd->slim, WCD9XXX_A_CLK_BUFF_EN1),
		reg_read(wcd->slim, WCD9XXX_A_CLK_BUFF_EN2),
		reg_read(wcd->slim, WCD9XXX_A_CDC_CLK_MCLK_CTL)
		); */

	/* reg_write(wcd->slim, WCD9XXX_A_CLK_BUFF_EN1, 0x05);
	reg_write(wcd->slim, WCD9XXX_A_CLK_BUFF_EN2, 0x04);
	reg_write(wcd->slim, WCD9XXX_A_CDC_CLK_MCLK_CTL, 1); */

	/*reg_read(wcd->slim, WCD9XXX_A_CHIP_CTL);
	reg_read(wcd->slim, WCD9XXX_A_CHIP_STATUS);
	reg_read(wcd->slim, WCD9XXX_A_CHIP_VERSION);
	reg_read(wcd->slim, WCD9XXX_A_CHIP_ID_BYTE_0);
	reg_read(wcd->slim, WCD9XXX_A_CHIP_ID_BYTE_1);
	reg_read(wcd->slim, WCD9XXX_A_CHIP_ID_BYTE_2);
	reg_read(wcd->slim, WCD9XXX_A_CHIP_ID_BYTE_3); */

	return ret;
}

static const struct wcd_slim_ch wcd9335_rx_chs[TAIKO_RX_MAX] = {
	WCD_SLIM_CH(16, 0),	 /* 16 */
	WCD_SLIM_CH(16 + 1, 1),	 /* 17 */
	WCD_SLIM_CH(16 + 2, 2),   /* 18 */
	WCD_SLIM_CH(16 + 3, 3),   /* 19 */
	WCD_SLIM_CH(16 + 4, 4),   /* 20 */
	WCD_SLIM_CH(16 + 5, 5),   /* 21 */
	WCD_SLIM_CH(16 + 6, 6),
	WCD_SLIM_CH(16 + 7, 7),
	WCD_SLIM_CH(16 + 8, 8),
	WCD_SLIM_CH(16 + 9, 9),
	WCD_SLIM_CH(16 + 10, 10),
	WCD_SLIM_CH(16 + 11, 11),
	WCD_SLIM_CH(16 + 12, 12),
};

static int wcd9335_slim_probe(struct slim_device *slim)
{
	struct device *dev = &slim->dev;
	struct wcd9335 *wcd;
	int ret = 0;

	printk("wcd9335_slim_probe\n");

	wcd = devm_kzalloc(dev, sizeof(*wcd), GFP_KERNEL);
	if (!wcd)
		return	-ENOMEM;

	wcd->slim = slim;
	wcd->dev = dev;

	ret = wcd9335_power_up(wcd);
	if (ret) {
		dev_err(dev, "Error parsing DT\n");
		return ret;
	}

	ret = wcd9335_parse_dt(wcd);

	ret = wcd9335_parse_slim_ifd_info(dev, &wcd->slim_ifd);
	if (ret) {
		dev_err(&slim->dev, "Error, parsing slim interface\n");
		return ret;
	}

	wcd->regmap = regmap_init_slimbus(slim, &wcd9320_regmap_config);

	if (IS_ERR(wcd->regmap)) {
		ret = PTR_ERR(wcd->regmap);
		dev_err(&slim->dev, "%s: Failed to allocate register map: %d\n",
			__func__, ret);
		return ret;
	}

	slim_set_clientdata(slim, wcd);

	usleep_range(600, 650);
	wcd9335_reset(wcd);

	ret = wcd9335_slim_get_laddr(wcd->slim, wcd->slim->e_addr,
				     ARRAY_SIZE(wcd->slim->e_addr),
				     &wcd->slim->laddr);
	if (ret) {
		pr_err("%s: failed to get slimbus %s logical address: %d\n",
		       __func__, wcd->slim->name, ret);
		ret = -EPROBE_DEFER;
		return ret;
	}

	wcd9335_init(wcd);

	ret = slim_add_device(slim->ctrl, &wcd->slim_ifd);
	if (ret) {
		pr_err("%s: error, adding SLIMBUS device failed\n", __func__);
		return ret;
	}

	ret = wcd9335_slim_get_laddr(&wcd->slim_ifd,
				     wcd->slim_ifd.e_addr,
				     ARRAY_SIZE(wcd->slim_ifd.e_addr),
				     &wcd->slim_ifd.laddr);
	if (ret) {
		pr_err("%s: failed to get slimbus %s logical address: %d\n",
		       __func__, wcd->slim->name, ret);
		ret = -EPROBE_DEFER;
		return ret;
	}

	wcd->ifd_regmap = regmap_init_slimbus(&wcd->slim_ifd, &wcd9320_ifd_regmap_config);
	if (IS_ERR(wcd->ifd_regmap)) {
		ret = PTR_ERR(wcd->ifd_regmap);
		dev_err(dev, "%s: Failed to allocate register map: %d\n",
			__func__, ret);
		return ret;
	}


	dev_set_drvdata(dev, wcd);
	//FIXME..

	ret = wcd9335_bring_up(wcd);
	if (ret) {
		pr_err("DEBUG:: %s: %d\n", __func__, __LINE__);
	}

	wcd->version = 2;
	wcd->dev = dev;
	wcd->slim_slave = &wcd->slim_ifd;

	wcd->slim_data.slim = slim;
	wcd->slim_data.slim_slave = &wcd->slim_ifd;
	//wcd->slim_data.regmap = wcd->regmap;
	wcd->slim_data.if_regmap = wcd->ifd_regmap;
	//FIXME
	wcd->slim_data.rx_port_ch_reg_base = 0x180 - (16 * 4);
	wcd->slim_data.port_rx_cfg_reg_base = 0x040 - 16;
	wcd->slim_data.port_tx_cfg_reg_base = 0x050;

	wcd->slim_data.rx_chs = devm_kzalloc(dev, sizeof(wcd9335_rx_chs), GFP_KERNEL);
	if (!wcd->slim_data.rx_chs)
		return -ENOMEM;

	memcpy(wcd->slim_data.rx_chs, wcd9335_rx_chs, sizeof(wcd9335_rx_chs));

	printk("tx_chs=%X\n", wcd->slim_data.tx_chs);

	ret = snd_soc_register_codec(dev, &taiko_codec_drv, taiko_dai, ARRAY_SIZE(taiko_dai));

	printk("wcd9335_slim_probe end %i\n", ret);

	return 0; //of_platform_populate(wcd->dev->of_node, NULL, NULL, wcd->dev);
}

static int wcd9335_slim_remove(struct slim_device *sdev)
{
	return 0;
}

static const struct slim_device_id wcd9335_slim_id[] = {
	{"taiko-slim-pgd", 0},
	{}
};

static struct slim_driver wcd9335_slim_driver = {
	.driver = {
		.name = "wcd9335-slim",
		.owner = THIS_MODULE,
	},
	.probe = wcd9335_slim_probe,
	.remove = wcd9335_slim_remove,
	.id_table = wcd9335_slim_id,
};

static int __init wcd_init(void)
{
	return  slim_driver_register(&wcd9335_slim_driver);
}
module_init(wcd_init);

static void __exit wcd_exit(void)
{
}
module_exit(wcd_exit);

MODULE_DESCRIPTION("Codec core driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
