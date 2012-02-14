/*
 * ASoC Audio Layer for Freescale STMP37XX/STMP378X ADC/DAC
 *
 * Author: Vladislav Buzov <vbuzov@embeddedalley.com>
 *
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/regs-audioin.h>
#include <mach/regs-audioout.h>
#include <mach/regs-pinctrl.h>
#include <mach/pinmux.h>
#include "stmp3xxx_pcm.h"

#define STMP3XXX_ADC_RATES	SNDRV_PCM_RATE_8000_192000
#define STMP3XXX_ADC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				SNDRV_PCM_FMTBIT_S32_LE)
#if 0
#define CHLOG(format, arg...)            \
        printk("stmp3xxx_dai.c - %s():%d - " format, __func__, __LINE__, ## arg)
#else
#define CHLOG(format, arg...)
#endif

extern int stmp3xxx_headphone_muted;
extern int stmp3xxx_playback_streams;
static int headphones_plugged;
static int timer_queued;
static void stmp3xxx_unmute(unsigned long ignored);
static void stmp3xxx_unmute_after(int msecs);

struct stmp3xxx_pcm_dma_params stmp3xxx_audio_in = {
	.name = "stmp3xxx adc",
	.dma_bus = STMP3XXX_BUS_APBX,
	.dma_ch	= 0,
	.irq = IRQ_ADC_DMA,
};

struct stmp3xxx_pcm_dma_params stmp3xxx_audio_out = {
	.name = "stmp3xxx dac",
	.dma_bus = STMP3XXX_BUS_APBX,
	.dma_ch	= 1,
	.irq = IRQ_DAC_DMA,
};

static void hpdetect_kick(void) {
    // If bit 8 is set, then the headphone has been unplugged.
    headphones_plugged = !!(HW_PINCTRL_DIN0_RD()&0x00000800);
	stmp3xxx_configure_irq(PINID_GPMI_D11, headphones_plugged?IRQ_TYPE_EDGE_FALLING:IRQ_TYPE_EDGE_RISING);
	CHLOG("Headphone detector kicked.  Current headphone state: %d\n", headphones_plugged);
}


static irqreturn_t hpdetect_handler(int irq, void *dev_id) {
    // If the headphone pin is detected as having fired the interrupt, deal
    // with the headphone event.

    // Mute or unmute the speakers, based on the status of headphones_plugged.
	CHLOG("Going to [un]mute speakers after 2msecs\n");
    stmp3xxx_unmute_after(2);

    return IRQ_HANDLED;
}


static irqreturn_t stmp3xxx_err_irq(int irq, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	u32 ctrl_reg;
	u32 overflow_mask;
	u32 underflow_mask;

	if (playback) {
		ctrl_reg = HW_AUDIOOUT_CTRL_RD();
		underflow_mask = BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ;
	} else {
		ctrl_reg = HW_AUDIOIN_CTRL_RD();
		underflow_mask = BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ;
		overflow_mask = BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ;
	}

	if (ctrl_reg & underflow_mask) {
		printk(KERN_DEBUG "%s underflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			HW_AUDIOOUT_CTRL_CLR(
				BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ);
		else
			HW_AUDIOIN_CTRL_CLR(
				BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ);

	} else if (ctrl_reg & overflow_mask) {
		printk(KERN_DEBUG "%s overflow detected\n",
		       playback ? "DAC" : "ADC");

		if (playback)
			HW_AUDIOOUT_CTRL_CLR(
				BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ);
		else
			HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ);
	} else
		printk(KERN_WARNING "Unknown DAC error interrupt\n");

	return IRQ_HANDLED;
}


// Unmutes the headphones.  Who would'a thunk?
// XXX Should we slowly ramp up the volume?  If the speakers still pop,
// then yes, we should take a look at that.
// If we still get headphone popping, take a look at the documentation for
// HW_AUDIOOUT_PWRDN.
static void unmute_headphones(void) {
    HW_AUDIOOUT_HPVOL_CLR(BM_AUDIOOUT_HPVOL_MUTE);
}

static void mute_headphones(void) {
    CHLOG("Muting headphones\n");
    HW_AUDIOOUT_HPVOL_SET(BM_AUDIOOUT_HPVOL_MUTE);
}

static void unmute_hard_speakers(void) {
    HW_AUDIOOUT_SPEAKERCTRL_CLR(BM_AUDIOOUT_SPEAKERCTRL_MUTE);
}

static void mute_hard_speakers(void) {
    HW_AUDIOOUT_SPEAKERCTRL_SET(BM_AUDIOOUT_SPEAKERCTRL_MUTE);
}



static void stmp3xxx_unmute(unsigned long ignored) {

    // Kick the headphone detection so we'll know whether to unmute
    // speakers or not.
    hpdetect_kick();

    // Unmute only if there are streams playing.
    if(stmp3xxx_playback_streams) {
        if(headphones_plugged) {
            CHLOG("Unmuting speakers\n");
            // Unmute the onboard speaker.  Used for hard-case models.
            unmute_hard_speakers();
        }
        else {
            CHLOG("Muting speakers\n");
            mute_hard_speakers();
        }

        if(!stmp3xxx_headphone_muted) {
            // Unmute the headphone pin.
            CHLOG("Unmuting headphones\n");
            unmute_headphones();
        }
        else
            CHLOG("Headphones appear muted\n");
    }
    else
        CHLOG("No playback streams found\n");
    timer_queued=0;
}

static void stmp3xxx_unmute_after(int msecs) {
    static struct timer_list timer;
    if(!timer_queued) {
        CHLOG("Unmuting speakers in %d msecs\n", msecs);
        timer_queued=1;

        init_timer(&timer);
        timer.data = 0;
        timer.function = stmp3xxx_unmute;
        timer.expires = jiffies + (HZ/1000)*msecs;
        add_timer(&timer);
    }
    else
        CHLOG("Not unmuting speakers, as there was already a request pending\n");
}

static int stmp3xxx_adc_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:

		if (playback) {
            // This will reset the audio system.  If we don't do this, there's
            // a nonzero chance the speaker will fail to start up.
            // Basically we're just writing random data to the audio
            // registers.
            HW_AUDIOOUT_DATA_WR(0x00000000);
            udelay(200);
            HW_AUDIOOUT_DATA_WR(0x00000001);
            udelay(200);

			HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_RUN);
            // XXX hacky: Rather than trying to traverse the substream and
            // turn it into a snd_codec, I just use an extern variable.  If
            // traversing this structure is easy, please do fix this.
            stmp3xxx_playback_streams++;

            CHLOG("Opened playback stream.  Will unmute after 2 msecs.\n");
            stmp3xxx_unmute_after(2);
        }
		else {
            CHLOG("Opened recording stream.\n");

            // XXX The FM radio calls arecord, and pipes it into aplay.
            // For some reason, the above block never gets called.
            // Therefore, unmute_after 2 msecs here.
            stmp3xxx_unmute_after(2);
			HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_RUN);
        }
		break;

	case SNDRV_PCM_TRIGGER_STOP:
		if (playback) {
            stmp3xxx_playback_streams--;
            if(stmp3xxx_playback_streams <= 0) {
                // Mute the speaker amps.
                mute_hard_speakers();
                mute_headphones();
                stmp3xxx_playback_streams = 0;
            }

            // Give the speakers / headphones time to mute before disabling
            // the audio chip.
            udelay(100);
			HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_RUN);
        }
		else
			HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_RUN);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
        CHLOG("Ignoring SNDRV_PCM_TRIGGER_RESUME\n");
        break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
        CHLOG("Ignoring SNDRV_PCM_TRIGGER_PAUSE_RELEASE\n");
        break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
        CHLOG("Ignoring SNDRV_PCM_TRIGGER_SUSPEND\n");
        break;

	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
        CHLOG("Ignoring SNDRV_PCM_TRIGGER_PAUSE_PUSH\n");
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}




static int stmp3xxx_adc_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;
	int irq;
	int ret;


	if (playback) {
		irq = IRQ_DAC_ERROR;
		cpu_dai->dma_data = &stmp3xxx_audio_out;

        if(ret)
            CHLOG("Unable to initialize IRQ for headphone detection: %d\n", ret);
	} else {
		irq = IRQ_ADC_ERROR;
		cpu_dai->dma_data = &stmp3xxx_audio_in;
	}

	ret = request_irq(irq, stmp3xxx_err_irq, 0, "STMP3xxx DAC/ADC Error",
			  substream);
	if (ret) {
		printk(KERN_ERR "%s: Unable to request ADC/DAC error irq %d\n",
		       __func__, IRQ_DAC_ERROR);
		return ret;
	}

	/* Enable error interrupt */
	if (playback) {
		HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_FIFO_OVERFLOW_IRQ);
		HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_FIFO_UNDERFLOW_IRQ);
		HW_AUDIOOUT_CTRL_SET(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN);

		/* Grab the pin we'll be using for headphone detect */
		stmp3xxx_request_pin(PINID_GPMI_D11, PIN_GPIO, "headphone detect");
		stmp3xxx_configure_irq_handler(PINID_GPMI_D11, hpdetect_handler, NULL);
		hpdetect_kick();
	} else {
		/* Set the audio recorder to use LRADC1, and set to an 8K resistor. */
		HW_AUDIOIN_MICLINE_SET(0x01300000);

		HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_OVERFLOW_IRQ);
		HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_UNDERFLOW_IRQ);
		HW_AUDIOIN_CTRL_SET(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN);
	}

	return 0;
}

static void stmp3xxx_adc_shutdown(struct snd_pcm_substream *substream)
{
	int playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 1 : 0;

	/* Disable error interrupt */
	if (playback) {
		HW_AUDIOOUT_CTRL_CLR(BM_AUDIOOUT_CTRL_FIFO_ERROR_IRQ_EN);
		free_irq(IRQ_DAC_ERROR, substream);
		stmp3xxx_configure_irq_handler(PINID_GPMI_D11, NULL, NULL);
		stmp3xxx_release_pin(PINID_GPMI_D11, "headphone detect");
	} else {
		HW_AUDIOIN_CTRL_CLR(BM_AUDIOIN_CTRL_FIFO_ERROR_IRQ_EN);
		free_irq(IRQ_ADC_ERROR, substream);
	}
}

#ifdef CONFIG_PM
static int stmp3xxx_adc_suspend(struct platform_device *pdev,
				struct snd_soc_dai *cpu_dai)
{
	return 0;
}

static int stmp3xxx_adc_resume(struct platform_device *pdev,
				struct snd_soc_dai *cpu_dai)
{
	return 0;
}
#else
#define stmp3xxx_adc_suspend	NULL
#define stmp3xxx_adc_resume	NULL
#endif /* CONFIG_PM */

struct snd_soc_dai stmp3xxx_adc_dai = {
	.name = "stmp3xxx adc/dac",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.suspend = stmp3xxx_adc_suspend,
	.resume = stmp3xxx_adc_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_ADC_RATES,
		.formats = STMP3XXX_ADC_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = STMP3XXX_ADC_RATES,
		.formats = STMP3XXX_ADC_FORMATS,
	},
	.ops = {
		.startup = stmp3xxx_adc_startup,
		.shutdown = stmp3xxx_adc_shutdown,
		.trigger = stmp3xxx_adc_trigger,
	},
};
EXPORT_SYMBOL_GPL(stmp3xxx_adc_dai);

MODULE_AUTHOR("Vladislav Buzov");
MODULE_DESCRIPTION("stmp3xxx dac/adc DAI");
MODULE_LICENSE("GPL");
