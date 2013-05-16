/*
 * tegra_wm8903.c - Tegra machine ASoC driver for boards using WM8903 codec.
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (C) 2010-2011 - NVIDIA, Inc.
 *
 * Based on code copyright/by:
 *
 * (c) 2009, 2010 Nvidia Graphics Pvt. Ltd.
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <asm/mach-types.h>

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <mach/tegra_wm8903_pdata.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#ifdef CONFIG_SND_SOC_FM34
#include <sound/fm34.h>
#include <sound/wm8903.h>
#endif

#include "../codecs/wm8903.h"

#include "tegra_pcm.h"
#include "tegra_asoc_utils.h"

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
#include "tegra20_das.h"
#endif

#define DRV_NAME "tegra-snd-wm8903"

#define GPIO_SPKR_EN    BIT(0)
#define GPIO_HP_MUTE    BIT(1)
#define GPIO_INT_MIC_EN BIT(2)
#define GPIO_EXT_MIC_EN BIT(3)
#define GPIO_HP_DET     BIT(4)
#define GPIO_EXT_HP_DET BIT(5)
#define GPIO_INT_SPKR_EN BIT(6)
#define GPIO_EXT_MIC_DET BIT(6)

#define AEC_DISABLE 0
#define AEC_ENABLE 1

#define SET_REG_VAL(r,m,l,v) (((r)&(~((m)<<(l))))|(((v)&(m))<<(l)))

enum audio_source {
    AUDIO_SOURCE_DEFAULT = 0,
    AUDIO_SOURCE_MIC = 1,
    AUDIO_SOURCE_VOICE_UPLINK = 2,
    AUDIO_SOURCE_VOICE_DOWNLINK = 3,
    AUDIO_SOURCE_VOICE_CALL = 4,
    AUDIO_SOURCE_CAMCORDER = 5,
    AUDIO_SOURCE_VOICE_RECOGNITION = 6,
    AUDIO_SOURCE_VOICE_COMMUNICATION = 7,
    AUDIO_SOURCE_MAX = AUDIO_SOURCE_VOICE_COMMUNICATION,

    AUDIO_SOURCE_LIST_END  // must be last - used to validate audio source type
};

enum audio_mode {
	MODE_INVALID = -2,
	MODE_CURRENT = -1,
	MODE_NORMAL = 0,
	MODE_RINGTONE,
	MODE_IN_CALL,
	MODE_IN_COMMUNICATION,
	NUM_MODES  // not a valid entry, denotes end-of-list
};

struct tegra_wm8903 {
	struct snd_soc_codec *codec;
	struct tegra_asoc_utils_data util_data;
	struct tegra_wm8903_platform_data *pdata;
	struct regulator *spk_reg;
	struct regulator *dmic_reg;
	int gpio_requested;
#ifdef CONFIG_SWITCH
	int jack_status;
#endif
	enum snd_soc_bias_level bias_level;
	int input_source;
	int voice_call;
	int audio_mode;
#ifdef CONFIG_SND_SOC_FM34
	struct snd_pcm_substream *cap_substream;
	struct mutex fm34_lock;
#endif
};

#ifdef CONFIG_SND_SOC_FM34
static void tegra_wm8903_digital_voip_gain(void *vcodec, int mode)
{
	struct snd_soc_codec *codec = vcodec;
	struct wm8903_platform_data *pdata_codec = dev_get_platdata(codec->dev);
	u16 dgt_val, spk_val;

	switch(mode) {
		case 0:
			dgt_val = 0xC0<<WM8903_ADCL_VOL_SHIFT;
			spk_val = pdata_codec->dac_speaker_volume + 3;
			break;
		default:
			dgt_val = pdata_codec->adc_digital_volume;
			spk_val = pdata_codec->dac_speaker_volume;
	}

	dgt_val |= WM8903_ADCVU;
	snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_LEFT, dgt_val);
	snd_soc_write(codec, WM8903_ADC_DIGITAL_VOLUME_RIGHT, dgt_val);
	spk_val |= WM8903_SPKVU;
	snd_soc_write(codec, WM8903_ANALOGUE_OUT3_LEFT, spk_val);
	snd_soc_write(codec, WM8903_ANALOGUE_OUT3_RIGHT, spk_val);
}

void tegra_pcm_control(int enable, struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	static int open_status = 0;
	static int aec_st = AEC_DISABLE;
	int func = substream->stream;

	mutex_lock(&machine->fm34_lock);
	if(enable) {
		open_status |= (0x01<<func);
		if(open_status == 0x03 && AEC_DISABLE == aec_st && snd_soc_dapm_get_pin_status(dapm, "Int Mic")
				&& machine->input_source != AUDIO_SOURCE_VOICE_RECOGNITION) {
			//printk(KERN_INFO "##### input:%d voice:%d mode:%d\n",
			//		machine->input_source, machine->voice_call, machine->audio_mode);
			if(machine->input_source == AUDIO_SOURCE_VOICE_COMMUNICATION
					|| machine->voice_call
					|| machine->audio_mode == MODE_IN_COMMUNICATION) {
				if(set_fM34_echo()) {
					aec_st = AEC_ENABLE;
					tegra_wm8903_digital_voip_gain(codec, 0);
				}
			}
		}
	} else {
		if(AEC_ENABLE == aec_st) {
			set_fM34_bypass(0);
			aec_st = AEC_DISABLE;
			tegra_wm8903_digital_voip_gain(codec, 255);
		}
		open_status &= ~(0x01<<func);
	}
	mutex_unlock(&machine->fm34_lock);
}
#else
void tegra_pcm_control(int enable, struct snd_pcm_substream *substream)
{
}
#endif

static int tegra_wm8903_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, i2s_daifmt;
	int err;
	struct clk *clk_m;
	int rate;
	int CtrlReg = 0;
	int VolumeCtrlReg = 0;

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 256 * srate;
		break;
	}



	clk_m = clk_get_sys(NULL, "clk_m");
	if (IS_ERR(clk_m)) {
		dev_err(card->dev, "Can't retrieve clk clk_m\n");
		err = PTR_ERR(clk_m);
		return err;
	}
	rate = clk_get_rate(clk_m);
	printk(KERN_INFO "tegra_wm8903:%s: rate %d\n", __func__, rate);

#if TEGRA30_I2S_MASTER_PLAYBACK
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
		     SND_SOC_DAIFMT_CBS_CFS;
#else
	mclk = rate;

	i2s_daifmt = SND_SOC_DAIFMT_NB_NF |
		     SND_SOC_DAIFMT_CBM_CFM;
#endif


	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	/* Use DSP mode for mono on Tegra20 */
	if ((params_channels(params) != 2) &&
	    (machine_is_ventana() || machine_is_harmony() ||
	    machine_is_kaen() || machine_is_aebl()))
		i2s_daifmt |= SND_SOC_DAIFMT_DSP_A;
	else
		i2s_daifmt |= SND_SOC_DAIFMT_I2S;

	err = snd_soc_dai_set_fmt(codec_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "codec_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_fmt(cpu_dai, i2s_daifmt);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC1,
					TEGRA20_DAS_DAP_ID_1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dap-dac path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_1,
					TEGRA20_DAS_DAP_SEL_DAC1);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif

        if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK) {
		/* Single Ended Mic */
		CtrlReg = WM8903_INL_CM_ENA |
			/* (0x0 << B00_MODE) | */
			(0x1 << WM8903_L_IP_SEL_N_SHIFT) |
			(0x1 << WM8903_L_IP_SEL_P_SHIFT);
		VolumeCtrlReg = 0x1B;
		/* Mic Setting */
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_1, CtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_1, CtrlReg);
		/* voulme for single ended mic */
		snd_soc_write(codec, WM8903_ANALOGUE_LEFT_INPUT_0,
				VolumeCtrlReg);
		snd_soc_write(codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
				VolumeCtrlReg);
		/* Left ADC data on both channels */
		CtrlReg = snd_soc_read(codec, WM8903_AUDIO_INTERFACE_0);
		CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, WM8903_AIFADCR_SRC_SHIFT, 0x0);
		CtrlReg  = SET_REG_VAL(CtrlReg, 0x1, WM8903_AIFADCL_SRC_SHIFT, 0x0);
		snd_soc_write(codec, WM8903_AUDIO_INTERFACE_0, CtrlReg);
		/* Enable analog inputs */
		CtrlReg = WM8903_INL_ENA | WM8903_INR_ENA;
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_0, CtrlReg);
		/* ADC Settings */
		/* CtrlReg = snd_soc_read(codec, WM8903_ADC_DIGITAL_0); */
		CtrlReg = (0x01<<WM8903_ADC_HPF_CUT_SHIFT) | WM8903_ADC_HPF_ENA;
		snd_soc_write(codec, WM8903_ADC_DIGITAL_0, CtrlReg);

		/* Disable sidetone */
		/* CtrlReg = 0; */
		/* snd_soc_write(codec, R20_SIDETONE_CTRL, CtrlReg); */

		/* Enable ADC */
		CtrlReg = snd_soc_read(codec, WM8903_POWER_MANAGEMENT_6);
		CtrlReg |= WM8903_ADCL_ENA;
		snd_soc_write(codec, WM8903_POWER_MANAGEMENT_6, CtrlReg);

	}

	#ifdef CONFIG_SND_SOC_FM34
	fm34_set_codec_link(codec, tegra_wm8903_digital_voip_gain);
	if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		machine->cap_substream = substream;
	#endif

	return 0;
}

static int tegra_bt_sco_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 64 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	err = snd_soc_dai_set_fmt(cpu_dai,
					SND_SOC_DAIFMT_DSP_A |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS);
	if (err < 0) {
		dev_err(card->dev, "cpu_dai fmt not set\n");
		return err;
	}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	err = tegra20_das_connect_dac_to_dap(TEGRA20_DAS_DAP_SEL_DAC2,
					TEGRA20_DAS_DAP_ID_4);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}

	err = tegra20_das_connect_dap_to_dac(TEGRA20_DAS_DAP_ID_4,
					TEGRA20_DAS_DAP_SEL_DAC2);
	if (err < 0) {
		dev_err(card->dev, "failed to set dac-dap path\n");
		return err;
	}
#endif
	return 0;
}

static int tegra_spdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	int srate, mclk, min_mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		mclk = 11289600;
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
		mclk = 12288000;
		break;
	default:
		return -EINVAL;
	}
	min_mclk = 128 * srate;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		if (!(machine->util_data.set_mclk % min_mclk))
			mclk = machine->util_data.set_mclk;
		else {
			dev_err(card->dev, "Can't configure clocks\n");
			return err;
		}
	}

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 1);

	return 0;
}

static int tegra_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(rtd->card);

	tegra_asoc_utils_lock_clk_rate(&machine->util_data, 0);

	return 0;
}

static struct snd_soc_ops tegra_wm8903_ops = {
	.hw_params = tegra_wm8903_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_wm8903_bt_sco_ops = {
	.hw_params = tegra_bt_sco_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_ops tegra_spdif_ops = {
	.hw_params = tegra_spdif_hw_params,
	.hw_free = tegra_hw_free,
};

static struct snd_soc_jack tegra_wm8903_hp_jack;
/* static struct snd_soc_jack tegra_wm8903_mic_jack; */
static struct snd_soc_jack tegra_wm8903_ext_hp_jack;

static struct snd_soc_jack_gpio tegra_wm8903_hp_jack_gpio = {
	.name = "headphone detect",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

static struct snd_soc_jack_gpio tegra_wm8903_ext_hp_jack_gpio = {
	.name = "external headphone detect",
	.report = SND_JACK_LINEOUT,
	.debounce_time = 150,
	.invert = 0,
};

#ifdef CONFIG_SWITCH
/* These values are copied from Android WiredAccessoryObserver */
enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

static struct switch_dev tegra_wm8903_headset_switch = {
	.name = "h2w",
};

static int tegra_wm8903_jack_notifier(struct notifier_block *self,
			      unsigned long action, void *dev)
{
	struct snd_soc_jack *jack = dev;
	struct snd_soc_codec *codec = jack->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	enum headset_state state = BIT_NO_HEADSET;
	int amic_status = 0;

	machine->jack_status &= ~SND_JACK_MICROPHONE;
	if (jack == &tegra_wm8903_hp_jack) {
		machine->jack_status &= ~SND_JACK_HEADPHONE;
		machine->jack_status |= (action ? SND_JACK_HEADPHONE : 0);

		if (action && (machine->jack_status & SND_JACK_LINEOUT)) {
			return NOTIFY_OK;
		}
	} else if (jack == &tegra_wm8903_ext_hp_jack) {
		machine->jack_status &= ~SND_JACK_LINEOUT;
		machine->jack_status |= (action ? SND_JACK_LINEOUT : 0);
	}

	if (! (machine->jack_status & SND_JACK_LINEOUT) && (machine->jack_status & SND_JACK_HEADPHONE)) {
		gpio_set_value(machine->pdata->gpio_ext_mic_en, 0);
		/* If we insert microphone too slow, sometimes device fail to detect mic.
		 * because of hw issue we can't use interrupt to reconginze microphone, we increase delay time to reduce fail rate. */
		msleep(550);
		amic_status = (! gpio_get_value(machine->pdata->gpio_ext_mic_det)) ? SND_JACK_MICROPHONE : 0;
		machine->jack_status |= (amic_status ? SND_JACK_MICROPHONE : 0);
		/*
		printk(KERN_INFO "tegra_wm8903:%s: A-Mic is %sdetected\n",
			__func__, amic_status ? "" : "NOT ");
		*/
	}

	if (! (machine->jack_status & SND_JACK_MICROPHONE)) {
		gpio_set_value(machine->pdata->gpio_ext_mic_en, 1);
	}

	switch (machine->jack_status) {
	case SND_JACK_HEADPHONE:
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_LINEOUT:
		state = BIT_HEADSET_NO_MIC;
		break;
	case (SND_JACK_LINEOUT | SND_JACK_HEADPHONE):
	case (SND_JACK_LINEOUT | SND_JACK_HEADSET):
		state = BIT_HEADSET_NO_MIC;
		break;
	case SND_JACK_HEADSET:
		state = BIT_HEADSET;
		break;
	case SND_JACK_MICROPHONE:
		/* mic: would not report */
	default:
		state = BIT_NO_HEADSET;
	}

	switch_set_state(&tegra_wm8903_headset_switch, state);

	return NOTIFY_OK;
}

static struct notifier_block tegra_wm8903_jack_detect_nb = {
	.notifier_call = tegra_wm8903_jack_notifier,
};
#else
static struct snd_soc_jack_pin tegra_wm8903_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_pin tegra_wm8903_mic_jack_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_jack_pin tegra_wm8903_ext_hp_jack_pins[] = {
	{
		.pin = "Ext HP Jack",
		.mask = SND_JACK_LINEOUT,
	},
};
#endif

static int tegra_wm8903_event_int_spk(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (machine->spk_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->spk_reg);
		else
			regulator_disable(machine->spk_reg);
	}

	if (!(machine->gpio_requested & GPIO_SPKR_EN))
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (! gpio_get_value(pdata->gpio_ext_hp_det)) {
			gpio_direction_output(machine->pdata->gpio_int_spkr_en, true);
			gpio_set_value_cansleep(pdata->gpio_spkr_en, true);
		}
		msleep(400);
	} else {
		gpio_direction_output(machine->pdata->gpio_int_spkr_en, false);
		gpio_set_value_cansleep(pdata->gpio_spkr_en, false);
	}

	return 0;
}

static int tegra_wm8903_event_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	if (!(machine->gpio_requested & GPIO_HP_MUTE))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_hp_mute,
				!SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_ext_hp(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;
	struct wm8903_platform_data *wpd_pdata = dev_get_platdata(w->codec->dev);
	int lineout_gain = 0;

	lineout_gain = WM8903_LINEOUTVU |
		(SND_SOC_DAPM_EVENT_ON(event) ?
		 wpd_pdata->dac_ext_hp_volume : wpd_pdata->dac_speaker_volume);

	snd_soc_write(w->codec, WM8903_ANALOGUE_OUT2_LEFT, lineout_gain);
	snd_soc_write(w->codec, WM8903_ANALOGUE_OUT2_RIGHT, lineout_gain);

	if (! SND_SOC_DAPM_EVENT_ON(event) &&
	    snd_soc_dapm_get_pin_status(dapm, "Headphone Jack")) {
		snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
		snd_soc_dapm_disable_pin(dapm, "Ext HP Jack");
		snd_soc_dapm_sync(dapm);
		return 0;
	}

	gpio_direction_output(pdata->gpio_int_spkr_en, ! SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static int tegra_wm8903_event_int_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->dmic_reg) {
		if (SND_SOC_DAPM_EVENT_ON(event))
			regulator_enable(machine->dmic_reg);
		else
			regulator_disable(machine->dmic_reg);
	}

	#ifdef CONFIG_SND_SOC_FM34
	tegra_pcm_control(SND_SOC_DAPM_EVENT_ON(event), machine->cap_substream);
	#endif

	/*
	if (!(machine->gpio_requested & GPIO_INT_MIC_EN))
		return 0;

	gpio_set_value_cansleep(pdata->gpio_int_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));
	*/

	return 0;
}

static int tegra_wm8903_event_ext_mic(struct snd_soc_dapm_widget *w,
					struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (!(machine->gpio_requested & GPIO_EXT_MIC_EN))
		return 0;

	/*
	gpio_set_value_cansleep(pdata->gpio_ext_mic_en,
				SND_SOC_DAPM_EVENT_ON(event));
	*/

	return 0;
}

static const struct snd_soc_dapm_widget cardhu_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_wm8903_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_wm8903_event_int_mic),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_widget tegra_wm8903_asics_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_LINE("Ext HP Jack", tegra_wm8903_event_ext_hp),
	SND_SOC_DAPM_MIC("Mic Jack", tegra_wm8903_event_ext_mic),
	SND_SOC_DAPM_MIC("Int Mic", tegra_wm8903_event_int_mic),
};

static const struct snd_soc_dapm_widget tegra_wm8903_default_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", tegra_wm8903_event_int_spk),
	SND_SOC_DAPM_HP("Headphone Jack", tegra_wm8903_event_hp),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route harmony_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route cardhu_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Line Out", NULL, "LINEOUTL"},
	{"Line Out", NULL, "LINEOUTR"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1L", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Int Mic"},
	{"IN1L", NULL, "Mic Bias"},
	{"IN1R", NULL, "Mic Bias"},
	{"IN3L", NULL, "Line In"},
	{"IN3R", NULL, "Line In"},
};

static const struct snd_soc_dapm_route seaboard_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route kaen_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "ROP"},
	{"Int Spk", NULL, "RON"},
	{"Int Spk", NULL, "LOP"},
	{"Int Spk", NULL, "LON"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN2R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route aebl_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "LINEOUTR"},
	{"Int Spk", NULL, "LINEOUTL"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_soc_dapm_route asics_audio_map[] = {
	{"Headphone Jack", NULL, "HPOUTR"},
	{"Headphone Jack", NULL, "HPOUTL"},
	{"Int Spk", NULL, "LINEOUTL"},
	{"Int Spk", NULL, "LINEOUTR"},
	{"Ext HP Jack", NULL, "LINEOUTL"},
	{"Ext HP Jack", NULL, "LINEOUTR"},
	{"Mic Bias", NULL, "Int Mic"},
	{"IN1R", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic Jack"},
	{"IN2L", NULL, "Mic Bias"},
	{"IN1R", NULL, "Mic Bias"},
};

static const struct snd_kcontrol_new cardhu_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("LineOut"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("LineIn"),
};

static const struct snd_kcontrol_new tegra_wm8903_asics_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Ext HP Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static const struct snd_kcontrol_new tegra_wm8903_default_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
};

static int tegra_wm8903_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = codec->card;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;
	int ret;

	machine->bias_level = SND_SOC_BIAS_STANDBY;
	machine->codec = codec;

	if (gpio_is_valid(pdata->gpio_spkr_en)) {
		ret = gpio_request(pdata->gpio_spkr_en, "spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_SPKR_EN;

		gpio_direction_output(pdata->gpio_spkr_en, 0);
	}

	if (gpio_is_valid(pdata->gpio_hp_mute)) {
		ret = gpio_request(pdata->gpio_hp_mute, "hp_mute");
		if (ret) {
			dev_err(card->dev, "cannot get hp_mute gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_HP_MUTE;

		gpio_direction_output(pdata->gpio_hp_mute, 0);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_en)) {
		ret = gpio_request(pdata->gpio_ext_mic_en, "ext_mic_en");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_EN;

		/* Enable ext mic; enable signal is active-low */
		gpio_direction_output(pdata->gpio_ext_mic_en, 1);
		gpio_export(pdata->gpio_ext_mic_en, false);
	}

	if (gpio_is_valid(pdata->gpio_ext_mic_det)) {
		ret = gpio_request(pdata->gpio_ext_mic_det, "ext_mic_det");
		if (ret) {
			dev_err(card->dev, "cannot get ext_mic_det gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_EXT_MIC_DET;

		gpio_direction_input(pdata->gpio_ext_mic_det);
		gpio_export(pdata->gpio_ext_mic_det, false);
	}

	if (machine_is_cardhu() || machine_is_ventana()) {
		ret = snd_soc_add_controls(codec, cardhu_controls,
				ARRAY_SIZE(cardhu_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm, cardhu_dapm_widgets,
				ARRAY_SIZE(cardhu_dapm_widgets));
	} else if (machine_is_antares() || machine_is_scorpio() ) {
		ret = snd_soc_add_controls(codec,
				tegra_wm8903_asics_controls,
				ARRAY_SIZE(tegra_wm8903_asics_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm,
				tegra_wm8903_asics_dapm_widgets,
				ARRAY_SIZE(tegra_wm8903_asics_dapm_widgets));
	} else {
		ret = snd_soc_add_controls(codec,
				tegra_wm8903_default_controls,
				ARRAY_SIZE(tegra_wm8903_default_controls));
		if (ret < 0)
			return ret;

		snd_soc_dapm_new_controls(dapm,
				tegra_wm8903_default_dapm_widgets,
				ARRAY_SIZE(tegra_wm8903_default_dapm_widgets));
	}

	if (machine_is_harmony()) {
		snd_soc_dapm_add_routes(dapm, harmony_audio_map,
					ARRAY_SIZE(harmony_audio_map));
	} else if (machine_is_cardhu() || machine_is_ventana()) {
		snd_soc_dapm_add_routes(dapm, cardhu_audio_map,
					ARRAY_SIZE(cardhu_audio_map));
	} else if (machine_is_seaboard()) {
		snd_soc_dapm_add_routes(dapm, seaboard_audio_map,
					ARRAY_SIZE(seaboard_audio_map));
	} else if (machine_is_kaen()) {
		snd_soc_dapm_add_routes(dapm, kaen_audio_map,
					ARRAY_SIZE(kaen_audio_map));
	} else if (machine_is_antares() || machine_is_scorpio() ){
		snd_soc_dapm_add_routes(dapm, asics_audio_map,
					ARRAY_SIZE(asics_audio_map));
	} else {
		snd_soc_dapm_add_routes(dapm, aebl_audio_map,
					ARRAY_SIZE(aebl_audio_map));
	}

	if (gpio_is_valid(pdata->gpio_int_spkr_en)) {
		ret = gpio_request(pdata->gpio_int_spkr_en, "int_spkr_en");
		if (ret) {
			dev_err(card->dev, "cannot get int_spkr_en gpio\n");
			return ret;
		}
		machine->gpio_requested |= GPIO_INT_SPKR_EN;

		gpio_direction_output(pdata->gpio_int_spkr_en, 1);
		gpio_export(pdata->gpio_int_spkr_en, false);
	} else
		printk(KERN_ERR "tegra_wm8903:%s: GPIO int_spkr_en is invalid\n", __func__);

	if (gpio_is_valid(pdata->gpio_ext_hp_det)) {
		tegra_wm8903_ext_hp_jack_gpio.gpio = pdata->gpio_ext_hp_det;
		snd_soc_jack_new(codec, "Ext HP Jack", SND_JACK_LINEOUT,
				&tegra_wm8903_ext_hp_jack);
		snd_soc_jack_add_gpios(&tegra_wm8903_ext_hp_jack,
					1,
					&tegra_wm8903_ext_hp_jack_gpio);
		machine->gpio_requested |= GPIO_EXT_HP_DET;
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_ext_hp_jack,
				ARRAY_SIZE(tegra_wm8903_ext_hp_jack_pins),
				tegra_wm8903_ext_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_ext_hp_jack,
				&tegra_wm8903_jack_detect_nb);
#endif
	}

	if (gpio_is_valid(pdata->gpio_hp_det)) {
		tegra_wm8903_hp_jack_gpio.gpio = pdata->gpio_hp_det;
		snd_soc_jack_new(codec, "Headphone Jack", SND_JACK_HEADPHONE,
				&tegra_wm8903_hp_jack);
#ifndef CONFIG_SWITCH
		snd_soc_jack_add_pins(&tegra_wm8903_hp_jack,
					ARRAY_SIZE(tegra_wm8903_hp_jack_pins),
					tegra_wm8903_hp_jack_pins);
#else
		snd_soc_jack_notifier_register(&tegra_wm8903_hp_jack,
					&tegra_wm8903_jack_detect_nb);
#endif
		snd_soc_jack_add_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);
		machine->gpio_requested |= GPIO_HP_DET;
	}

#if 0
	snd_soc_jack_new(codec, "Mic Jack", SND_JACK_MICROPHONE,
			 &tegra_wm8903_mic_jack);
#ifndef CONFIG_SWITCH
	snd_soc_jack_add_pins(&tegra_wm8903_mic_jack,
			      ARRAY_SIZE(tegra_wm8903_mic_jack_pins),
			      tegra_wm8903_mic_jack_pins);
#else
	snd_soc_jack_notifier_register(&tegra_wm8903_mic_jack,
				&tegra_wm8903_jack_detect_nb);
#endif
	wm8903_mic_detect(codec, &tegra_wm8903_mic_jack, SND_JACK_MICROPHONE,
			  machine_is_cardhu() ? SND_JACK_MICROPHONE : 0);
	snd_soc_dapm_force_enable_pin(dapm, "Mic Bias");

	/* FIXME: Calculate automatically based on DAPM routes? */
	if (!machine_is_harmony() && !machine_is_ventana() &&
	    !machine_is_cardhu())
		snd_soc_dapm_nc_pin(dapm, "IN1L");
	if (!machine_is_seaboard() && !machine_is_aebl() &&
	    !machine_is_cardhu())
		snd_soc_dapm_nc_pin(dapm, "IN1R");
	snd_soc_dapm_nc_pin(dapm, "IN2L");
	if (!machine_is_kaen())
		snd_soc_dapm_nc_pin(dapm, "IN2R");
	snd_soc_dapm_nc_pin(dapm, "IN3L");
	snd_soc_dapm_nc_pin(dapm, "IN3R");

	if (machine_is_aebl()) {
		snd_soc_dapm_nc_pin(dapm, "LON");
		snd_soc_dapm_nc_pin(dapm, "RON");
		snd_soc_dapm_nc_pin(dapm, "ROP");
		snd_soc_dapm_nc_pin(dapm, "LOP");
	} else {
		snd_soc_dapm_nc_pin(dapm, "LINEOUTR");
		snd_soc_dapm_nc_pin(dapm, "LINEOUTL");
	}
#endif

	snd_soc_dapm_disable_pin(dapm, "Int Spk");
	snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_disable_pin(dapm, "Ext HP Jack");
	snd_soc_dapm_disable_pin(dapm, "Mic Jack");
	snd_soc_dapm_disable_pin(dapm, "Int Mic");

	snd_soc_dapm_sync(dapm);

	return 0;
}

#ifndef CONFIG_ARCH_TEGRA_2x_SOC
static int tegra30_soc_set_bias_level(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level == SND_SOC_BIAS_OFF &&
		level != SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_enable(&machine->util_data);

	return 0;
}

static int tegra30_soc_set_bias_level_post(struct snd_soc_card *card,
					enum snd_soc_bias_level level)
{
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if (machine->bias_level != SND_SOC_BIAS_OFF &&
		level == SND_SOC_BIAS_OFF)
		tegra_asoc_utils_clk_disable(&machine->util_data);

	machine->bias_level = level;

	return 0 ;
}
#endif

static struct snd_soc_dai_link tegra_wm8903_dai[] = {
	{
		.name = "WM8903",
		.stream_name = "WM8903 PCM",
		.codec_name = "wm8903.0-001a",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.0",
		.codec_dai_name = "wm8903-hifi",
		.init = tegra_wm8903_init,
		.ops = &tegra_wm8903_ops,
	},
	{
		.name = "SPDIF",
		.stream_name = "SPDIF PCM",
		.codec_name = "spdif-dit.0",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-spdif",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_spdif_ops,
	},
	{
		.name = "BT-SCO",
		.stream_name = "BT SCO PCM",
		.codec_name = "spdif-dit.1",
		.platform_name = "tegra-pcm-audio",
		.cpu_dai_name = "tegra20-i2s.1",
		.codec_dai_name = "dit-hifi",
		.ops = &tegra_wm8903_bt_sco_ops,
	},
};

int tegra_wm8903_suspend_pre(struct snd_soc_card *card)
{
	/*
	struct snd_soc_jack_gpio *gpio = &tegra_wm8903_hp_jack_gpio;

	if (gpio_is_valid(gpio->gpio))
		disable_irq(gpio_to_irq(gpio->gpio));
	*/

	return 0;
}

int tegra_wm8903_resume_post(struct snd_soc_card *card)
{
	int val;
	struct snd_soc_jack_gpio *hp_gpio = &tegra_wm8903_hp_jack_gpio;
	struct snd_soc_jack_gpio *ext_hp_gpio = &tegra_wm8903_ext_hp_jack_gpio;

	if (gpio_is_valid(ext_hp_gpio->gpio)) {
		val = gpio_get_value(ext_hp_gpio->gpio);
		val = ext_hp_gpio->invert ? !val : val;
		val = val ? ext_hp_gpio->report : 0;
		snd_soc_jack_report(ext_hp_gpio->jack, val, ext_hp_gpio->report);
		/* enable_irq(gpio_to_irq(ext_hp_gpio->gpio)); */
	}

	if (gpio_is_valid(hp_gpio->gpio)) {
		val = gpio_get_value(hp_gpio->gpio);
		val = hp_gpio->invert ? !val : val;
		val = val ? hp_gpio->report : 0;
		snd_soc_jack_report(hp_gpio->jack, val, hp_gpio->report);
		/* enable_irq(gpio_to_irq(hp_gpio->gpio)); */
	}

	return 0;
}

static struct snd_soc_card snd_soc_tegra_wm8903 = {
	.name = "tegra-wm8903",
	.dai_link = tegra_wm8903_dai,
	.num_links = ARRAY_SIZE(tegra_wm8903_dai),
	.suspend_pre = tegra_wm8903_suspend_pre,
	.resume_post = tegra_wm8903_resume_post,
	/* .set_bias_level = tegra30_soc_set_bias_level, */
	/* .set_bias_level_post = tegra30_soc_set_bias_level_post, */
};

ssize_t jack_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "0x%X\n", machine->jack_status);
	return strlen(buf);
}
DEVICE_ATTR(jack_status, S_IRUSR | S_IRGRP | S_IROTH, jack_status_show, NULL);

ssize_t input_source_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if(len != 2){
		return -EINVAL;
	}

	machine->input_source = *buf-'0';
	return len;
}

ssize_t input_source_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%d\n", machine->input_source);
	return strlen(buf);
}
DEVICE_ATTR(input_source, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, input_source_show, input_source_store);

ssize_t voice_call_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	if(*buf == '+')
		machine->voice_call++;
	else if(*buf == '-' && machine->voice_call)
		machine->voice_call--;
	return len;
}

ssize_t voice_call_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%d\n", machine->voice_call);
	return strlen(buf);
}
DEVICE_ATTR(voice_call, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, voice_call_show, voice_call_store);

ssize_t audio_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	machine->audio_mode = *buf-'0';
	return len;
}

ssize_t audio_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);

	sprintf(buf, "%d\n", machine->audio_mode);
	return strlen(buf);
}
DEVICE_ATTR(audio_mode, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, audio_mode_show, audio_mode_store);

static __devinit int tegra_wm8903_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_wm8903;
	struct tegra_wm8903 *machine;
	struct tegra_wm8903_platform_data *pdata;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	machine = kzalloc(sizeof(struct tegra_wm8903), GFP_KERNEL);
	if (!machine) {
		dev_err(&pdev->dev, "Can't allocate tegra_wm8903 struct\n");
		return -ENOMEM;
	}

	machine->pdata = pdata;
#ifdef CONFIG_SND_SOC_FM34
	mutex_init(&machine->fm34_lock);
#endif

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret)
		goto err_free_machine;

	machine->spk_reg = regulator_get(&pdev->dev, "vdd_spk_amp");
	if (IS_ERR(machine->spk_reg)) {
		dev_info(&pdev->dev, "No speaker regulator found\n");
		machine->spk_reg = 0;
	}

	machine->dmic_reg = regulator_get(&pdev->dev, "vdd_dmic");
	if (IS_ERR(machine->dmic_reg)) {
		dev_info(&pdev->dev, "No digital mic regulator found\n");
		machine->dmic_reg = 0;
	}

	if (machine_is_cardhu()) {
		tegra_wm8903_dai[0].codec_name = "wm8903.4-001a",
		tegra_wm8903_dai[0].cpu_dai_name = "tegra30-i2s.1";

		tegra_wm8903_dai[1].cpu_dai_name = "tegra30-spdif";

		tegra_wm8903_dai[2].cpu_dai_name = "tegra30-i2s.3";
	}

#ifdef CONFIG_SWITCH
	/* Addd h2w swith class support */
	ret = switch_dev_register(&tegra_wm8903_headset_switch);
	if (ret < 0)
		goto err_fini_utils;
#endif

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_switch;
	}

	if (!card->instantiated) {
		ret = -ENODEV;
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		goto err_unregister_card;
	}

	ret = device_create_file(card->dev, &dev_attr_jack_status);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry jack_status failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_input_source);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry input_source failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_voice_call);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry output_source failed\n", __FUNCTION__);
	}

	ret = device_create_file(card->dev, &dev_attr_audio_mode);
	if (ret) {
		dev_err(card->dev, "%s: add_sysfs_entry audio_mode failed\n", __FUNCTION__);
	}

	return 0;

err_unregister_card:
	snd_soc_unregister_card(card);
err_unregister_switch:
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_wm8903_headset_switch);
err_fini_utils:
#endif
	tegra_asoc_utils_fini(&machine->util_data);
err_free_machine:
	kfree(machine);
	return ret;
}

static int __devexit tegra_wm8903_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct tegra_wm8903_platform_data *pdata = machine->pdata;

	device_remove_file(card->dev, &dev_attr_jack_status);
	device_remove_file(card->dev, &dev_attr_input_source);
	device_remove_file(card->dev, &dev_attr_voice_call);
	device_remove_file(card->dev, &dev_attr_audio_mode);

	if (machine->gpio_requested & GPIO_HP_DET)
		snd_soc_jack_free_gpios(&tegra_wm8903_hp_jack,
					1,
					&tegra_wm8903_hp_jack_gpio);
	if (machine->gpio_requested & GPIO_EXT_MIC_EN)
		gpio_free(pdata->gpio_ext_mic_en);
	if (machine->gpio_requested & GPIO_EXT_MIC_DET)
		gpio_free(pdata->gpio_ext_mic_det);
	if (machine->gpio_requested & GPIO_HP_MUTE)
		gpio_free(pdata->gpio_hp_mute);
	if (machine->gpio_requested & GPIO_SPKR_EN)
		gpio_free(pdata->gpio_spkr_en);
	machine->gpio_requested = 0;

	if (machine->spk_reg)
		regulator_put(machine->spk_reg);
	if (machine->dmic_reg)
		regulator_put(machine->dmic_reg);

	snd_soc_unregister_card(card);

	tegra_asoc_utils_fini(&machine->util_data);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&tegra_wm8903_headset_switch);
#endif
	kfree(machine);

	return 0;
}

static void tegra_wm8903_driver_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_wm8903 *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_codec *codec = machine->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_disable_pin(dapm, "Int Spk");
	snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_disable_pin(dapm, "Ext HP Jack");
	snd_soc_dapm_sync(dapm);
}

static struct platform_driver tegra_wm8903_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe = tegra_wm8903_driver_probe,
	.remove = __devexit_p(tegra_wm8903_driver_remove),
	.shutdown = tegra_wm8903_driver_shutdown,
};

static int __init tegra_wm8903_modinit(void)
{
	return platform_driver_register(&tegra_wm8903_driver);
}
module_init(tegra_wm8903_modinit);

static void __exit tegra_wm8903_modexit(void)
{
	platform_driver_unregister(&tegra_wm8903_driver);
}
module_exit(tegra_wm8903_modexit);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra+WM8903 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
