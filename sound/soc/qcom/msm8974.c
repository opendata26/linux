// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, Linaro Limited
 */

#include <linux/soc/qcom/apr.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>

static int msm8996_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
					      struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	return 0;
}

static int apq8096_audrx_init(struct snd_soc_pcm_runtime *rtd)
{
	int err;
	void *config_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	void *mbhc_calibration;
	struct snd_card *card;
	struct snd_info_entry *entry;
	struct msm8996_asoc_mach_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

	printk("audrx init\n");

	/* Codec SLIMBUS configuration
	 * RX1, RX2, RX3, RX4, RX5, RX6, RX7, RX8, RX9, RX10, RX11, RX12, RX13
	 * TX1, TX2, TX3, TX4, TX5, TX6, TX7, TX8, TX9, TX10, TX11, TX12, TX13
	 * TX14, TX15, TX16
	 */
	unsigned int rx_ch[] = {144, 145};
	unsigned int tx_ch[] = {};

	/*snd_soc_dapm_add_routes(dapm, wcd9335_audio_paths,
				ARRAY_SIZE(wcd9335_audio_paths));
	snd_soc_dapm_sync(dapm); */

	snd_soc_dai_set_channel_map(codec_dai, 0,
				    tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	snd_soc_dai_set_channel_map(cpu_dai, 0,
				    tx_ch, ARRAY_SIZE(rx_ch), rx_ch);

	return 0;
out:
	return err;
}


static int apq8096_sbc_parse_of(struct snd_soc_card *card)
{
	struct device *dev = card->dev;
	struct snd_soc_dai_link *link;
	struct device_node *np, *codec, *platform, *cpu, *node  = dev->of_node;
	int ret, num_links;
	bool is_fe;


	ret = snd_soc_of_parse_card_name(card, "qcom,model");
	if (ret)
		dev_err(dev, "Error parsing card name: %d\n", ret);

	if (of_property_read_bool(dev->of_node, "qcom,audio-routing"))
		ret = snd_soc_of_parse_audio_routing(card,
					"qcom,audio-routing");

	/* Populate links */
	num_links = of_get_child_count(node);

	dev_info(dev, "Found %d child audio dai links..\n", num_links);
	/* Allocate the private data and the DAI link array */
	card->dai_link = devm_kzalloc(dev, sizeof(*link) * num_links,
			    GFP_KERNEL);
	if (!card->dai_link)
		return -ENOMEM;

	card->num_links	= num_links;

	link = card->dai_link;

	for_each_child_of_node(node, np) {
		is_fe = false;
		if (of_property_read_bool(np, "is-fe"))
			is_fe = true;

		if (is_fe) {
			/* BE is dummy */
			link->codec_of_node	= NULL;
			link->codec_dai_name	= "snd-soc-dummy-dai";
			link->codec_name	= "snd-soc-dummy";

			/* FE settings */
			link->dynamic		= 1;
			link->dpcm_playback = 1;

		} else {
			link->no_pcm = 1;
			link->dpcm_playback = 1;
			link->ignore_suspend = 1;
			link->ignore_pmdown_time = 1;
			link->be_hw_params_fixup = msm8996_be_hw_params_fixup;
			link->init = apq8096_audrx_init;
		}

		cpu = of_get_child_by_name(np, "cpu");
		platform = of_get_child_by_name(np, "platform");
		codec = of_get_child_by_name(np, "codec");

		if (!cpu) {
			dev_err(dev, "Can't find cpu DT node\n");
			return -EINVAL;
		}

		link->cpu_of_node = of_parse_phandle(cpu, "sound-dai", 0);
		if (!link->cpu_of_node) {
			dev_err(card->dev, "error getting cpu phandle\n");
			return -EINVAL;
		}

		link->platform_of_node = of_parse_phandle(platform,
							  "sound-dai", 0);
		if (!link->platform_of_node) {
			dev_err(card->dev, "error getting platform phandle\n");
			return -EINVAL;
		}

		ret = snd_soc_of_get_dai_name(cpu, &link->cpu_dai_name);
		if (ret) {
			dev_err(card->dev, "error getting cpu dai name\n");
			return ret;
		}

		if (codec) {
			ret = snd_soc_of_get_dai_link_codecs(dev, codec, link);

			if (ret < 0) {
				dev_err(card->dev, "error getting codec dai name\n");
				return ret;
			}
		}

		ret = of_property_read_string(np, "link-name", &link->name);
		if (ret) {
			dev_err(card->dev, "error getting codec dai_link name\n");
			return ret;
		}

		link->stream_name = link->name;
		link++;
	}

	return ret;
}

static int msm_snd_apq8096_probe(struct apr_device *adev)
{
	int ret;
	struct snd_soc_card *card;

	card = devm_kzalloc(&adev->dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->dev = &adev->dev;

	ret = apq8096_sbc_parse_of(card);
	if (ret) {
		dev_err(&adev->dev, "Error parsing OF data\n");
		return ret;
	}

	ret = devm_snd_soc_register_card(&adev->dev, card);
	if (ret)
		dev_err(&adev->dev, "sound card register failed (%d)!\n", ret);
	else
		dev_err(&adev->dev, "sound card register Sucessfull\n");

	return ret;
}

static const struct of_device_id msm_snd_apq8096_dt_match[] = {
	{.compatible = "qcom,apq8096-sndcard"},
	{}
};

static struct apr_driver msm_snd_apq8096_driver = {
	.probe  = msm_snd_apq8096_probe,
	.driver = {
		.name = "msm-snd-apq8096",
		.owner = THIS_MODULE,
		.of_match_table = msm_snd_apq8096_dt_match,
	},
};
module_apr_driver(msm_snd_apq8096_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("APQ8096 ASoC Machine Driver");
MODULE_LICENSE("GPL v2");
