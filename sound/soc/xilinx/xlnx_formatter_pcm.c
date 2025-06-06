// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx ASoC audio formatter support
 *
 * Copyright (C) 2018 Xilinx, Inc.
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>

#include <sound/asoundef.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "xlnx_snd_common.h"

#define DRV_NAME "xlnx_formatter_pcm"

#define XLNX_S2MM_OFFSET	0
#define XLNX_MM2S_OFFSET	0x100

#define XLNX_AUD_CORE_CONFIG	0x4
#define XLNX_AUD_CTRL		0x10
#define XLNX_AUD_STS		0x14

#define AUD_CTRL_RESET_MASK	BIT(1)
#define AUD_CFG_MM2S_MASK	BIT(15)
#define AUD_CFG_S2MM_MASK	BIT(31)

#define XLNX_AUD_FS_MULTIPLIER	0x18
#define XLNX_AUD_PERIOD_CONFIG	0x1C
#define XLNX_AUD_BUFF_ADDR_LSB	0x20
#define XLNX_AUD_BUFF_ADDR_MSB	0x24
#define XLNX_AUD_XFER_COUNT	0x28
#define XLNX_AUD_CH_STS_START	0x2C
#define XLNX_BYTES_PER_CH	0x44
#define XLNX_AUD_ALIGN_BYTES	64

#define AUD_STS_IOC_IRQ_MASK	BIT(31)
#define AUD_STS_CH_STS_MASK	BIT(29)
#define AUD_CTRL_IOC_IRQ_MASK	BIT(13)
#define AUD_CTRL_TOUT_IRQ_MASK	BIT(14)
#define AUD_CTRL_DMA_EN_MASK	BIT(0)

#define CFG_MM2S_CH_MASK	GENMASK(11, 8)
#define CFG_MM2S_CH_SHIFT	8
#define CFG_MM2S_XFER_MASK	GENMASK(14, 13)
#define CFG_MM2S_XFER_SHIFT	13
#define CFG_MM2S_PKG_MASK	BIT(12)

#define CFG_S2MM_CH_MASK	GENMASK(27, 24)
#define CFG_S2MM_CH_SHIFT	24
#define CFG_S2MM_XFER_MASK	GENMASK(30, 29)
#define CFG_S2MM_XFER_SHIFT	29
#define CFG_S2MM_PKG_MASK	BIT(28)

#define AUD_CTRL_DATA_WIDTH_MASK	GENMASK(18, 16)
#define AUD_CTRL_DATA_WIDTH_SHIFT	16
#define AUD_CTRL_ACTIVE_CH_MASK		GENMASK(22, 19)
#define AUD_CTRL_ACTIVE_CH_SHIFT	19
#define PERIOD_CFG_PERIODS_SHIFT	16

#define PERIODS_MIN		2
#define PERIODS_MAX		6
#define PERIOD_BYTES_MIN	192
#define PERIOD_BYTES_MAX	(50 * 1024)
#define XLNX_PARAM_UNKNOWN	0

enum bit_depth {
	BIT_DEPTH_8,
	BIT_DEPTH_16,
	BIT_DEPTH_20,
	BIT_DEPTH_24,
	BIT_DEPTH_32,
};

struct xlnx_pcm_drv_data {
	void __iomem *mmio;
	bool s2mm_presence;
	bool mm2s_presence;
	int s2mm_irq;
	int mm2s_irq;
	struct snd_pcm_substream *play_stream;
	struct snd_pcm_substream *capture_stream;
	struct platform_device *pdev;
	struct device_node *nodes[XLNX_MAX_PATHS];
	struct clk *axi_clk;
	struct clk *mm2s_axis_clk;
	struct clk *s2mm_axis_clk;
	struct clk *aud_mclk;
	unsigned int sysclk;
};

/*
 * struct xlnx_pcm_stream_param - stream configuration
 * @mmio: base address offset
 * @interleaved: audio channels arrangement in buffer
 * @xfer_mode: data formatting mode during transfer
 * @ch_limit: Maximum channels supported
 * @buffer_size: stream ring buffer size
 */
struct xlnx_pcm_stream_param {
	void __iomem *mmio;
	bool interleaved;
	u32 xfer_mode;
	u32 ch_limit;
	u64 buffer_size;
};

static const struct snd_pcm_hardware xlnx_pcm_hardware = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH | SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE |
		   SNDRV_PCM_FMTBIT_S24_LE,
	.channels_min = 2,
	.channels_max = 8,
	.rates = SNDRV_PCM_RATE_8000_192000,
	.rate_min = 8000,
	.rate_max = 192000,
	.buffer_bytes_max = PERIODS_MAX * PERIOD_BYTES_MAX,
	.period_bytes_min = PERIOD_BYTES_MIN,
	.period_bytes_max = PERIOD_BYTES_MAX,
	.periods_min = PERIODS_MIN,
	.periods_max = PERIODS_MAX,
};

enum {
	AES_TO_AES,
	AES_TO_PCM,
	PCM_TO_PCM,
	PCM_TO_AES
};

static void xlnx_parse_aes_params(u32 chsts_reg1_val, u32 chsts_reg2_val,
				  struct device *dev)
{
	u32 padded, srate, bit_depth, status[2];

	if (chsts_reg1_val & IEC958_AES0_PROFESSIONAL) {
		status[0] = chsts_reg1_val & 0xff;
		status[1] = (chsts_reg1_val >> 16) & 0xff;

		switch (status[0] & IEC958_AES0_PRO_FS) {
		case IEC958_AES0_PRO_FS_44100:
			srate = 44100;
			break;
		case IEC958_AES0_PRO_FS_48000:
			srate = 48000;
			break;
		case IEC958_AES0_PRO_FS_32000:
			srate = 32000;
			break;
		case IEC958_AES0_PRO_FS_NOTID:
		default:
			srate = XLNX_PARAM_UNKNOWN;
			break;
		}

		switch (status[1] & IEC958_AES2_PRO_SBITS) {
		case IEC958_AES2_PRO_WORDLEN_NOTID:
		case IEC958_AES2_PRO_SBITS_20:
			padded = 0;
			break;
		case IEC958_AES2_PRO_SBITS_24:
			padded = 4;
			break;
		default:
			bit_depth = XLNX_PARAM_UNKNOWN;
			goto log_params;
		}

		switch (status[1] & IEC958_AES2_PRO_WORDLEN) {
		case IEC958_AES2_PRO_WORDLEN_20_16:
			bit_depth = 16 + padded;
			break;
		case IEC958_AES2_PRO_WORDLEN_22_18:
			bit_depth = 18 + padded;
			break;
		case IEC958_AES2_PRO_WORDLEN_23_19:
			bit_depth = 19 + padded;
			break;
		case IEC958_AES2_PRO_WORDLEN_24_20:
			bit_depth = 20 + padded;
			break;
		case IEC958_AES2_PRO_WORDLEN_NOTID:
		default:
			bit_depth = XLNX_PARAM_UNKNOWN;
			break;
		}

	} else {
		status[0] = (chsts_reg1_val >> 24) & 0xff;
		status[1] = chsts_reg2_val & 0xff;

		switch (status[0] & IEC958_AES3_CON_FS) {
		case IEC958_AES3_CON_FS_44100:
			srate = 44100;
			break;
		case IEC958_AES3_CON_FS_48000:
			srate = 48000;
			break;
		case IEC958_AES3_CON_FS_32000:
			srate = 32000;
			break;
		default:
			srate = XLNX_PARAM_UNKNOWN;
			break;
		}

		if (status[1] & IEC958_AES4_CON_MAX_WORDLEN_24)
			padded = 4;
		else
			padded = 0;

		switch (status[1] & IEC958_AES4_CON_WORDLEN) {
		case IEC958_AES4_CON_WORDLEN_20_16:
			bit_depth = 16 + padded;
			break;
		case IEC958_AES4_CON_WORDLEN_22_18:
			bit_depth = 18 + padded;
			break;
		case IEC958_AES4_CON_WORDLEN_23_19:
			bit_depth = 19 + padded;
			break;
		case IEC958_AES4_CON_WORDLEN_24_20:
			bit_depth = 20 + padded;
			break;
		case IEC958_AES4_CON_WORDLEN_21_17:
			bit_depth = 17 + padded;
			break;
		case IEC958_AES4_CON_WORDLEN_NOTID:
		default:
			bit_depth = XLNX_PARAM_UNKNOWN;
			break;
		}
	}

log_params:
	if (srate != XLNX_PARAM_UNKNOWN)
		dev_info(dev, "sample rate = %d\n", srate);
	else
		dev_info(dev, "sample rate = unknown\n");

	if (bit_depth != XLNX_PARAM_UNKNOWN)
		dev_info(dev, "bit_depth = %d\n", bit_depth);
	else
		dev_info(dev, "bit_depth = unknown\n");
}

static int xlnx_formatter_pcm_reset(void __iomem *mmio_base)
{
	u32 val, retries = 0;

	val = ioread32(mmio_base + XLNX_AUD_CTRL);
	val |= AUD_CTRL_RESET_MASK;
	iowrite32(val, mmio_base + XLNX_AUD_CTRL);

	val = ioread32(mmio_base + XLNX_AUD_CTRL);
	/* Poll for maximum timeout of approximately 100ms (1 * 100)*/
	while ((val & AUD_CTRL_RESET_MASK) && (retries < 100)) {
		mdelay(1);
		retries++;
		val = ioread32(mmio_base + XLNX_AUD_CTRL);
	}
	if (val & AUD_CTRL_RESET_MASK)
		return -ENODEV;

	return 0;
}

static void xlnx_formatter_disable_irqs(void __iomem *mmio_base, int stream)
{
	u32 val;

	val = ioread32(mmio_base + XLNX_AUD_CTRL);
	val &= ~AUD_CTRL_IOC_IRQ_MASK;
	if (stream == SNDRV_PCM_STREAM_CAPTURE)
		val &= ~AUD_CTRL_TOUT_IRQ_MASK;

	iowrite32(val, mmio_base + XLNX_AUD_CTRL);
}

static irqreturn_t xlnx_mm2s_irq_handler(int irq, void *arg)
{
	u32 val;
	void __iomem *reg;
	struct device *dev = arg;
	struct xlnx_pcm_drv_data *adata = dev_get_drvdata(dev);

	reg = adata->mmio + XLNX_MM2S_OFFSET + XLNX_AUD_STS;
	val = ioread32(reg);
	if (val & AUD_STS_IOC_IRQ_MASK) {
		iowrite32(val & AUD_STS_IOC_IRQ_MASK, reg);
		if (adata->play_stream)
			snd_pcm_period_elapsed(adata->play_stream);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t xlnx_s2mm_irq_handler(int irq, void *arg)
{
	u32 val;
	void __iomem *reg;
	struct device *dev = arg;
	struct xlnx_pcm_drv_data *adata = dev_get_drvdata(dev);

	reg = adata->mmio + XLNX_S2MM_OFFSET + XLNX_AUD_STS;
	val = ioread32(reg);
	if (val & AUD_STS_IOC_IRQ_MASK) {
		iowrite32(val & AUD_STS_IOC_IRQ_MASK, reg);
		if (adata->capture_stream)
			snd_pcm_period_elapsed(adata->capture_stream);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int xlnx_formatter_set_sysclk(struct snd_soc_component *component,
				     int clk_id, int source, unsigned int freq, int dir)
{
	struct xlnx_pcm_drv_data *adata = dev_get_drvdata(component->dev);

	adata->sysclk = freq;
	return 0;
}

static int xlnx_formatter_pcm_open(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	int err;
	u32 val, data_format_mode;
	u32 ch_count_mask, ch_count_shift, data_xfer_mode, data_xfer_shift;
	struct xlnx_pcm_stream_param *stream_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xlnx_pcm_drv_data *adata;

	if (!component)
		return -ENODEV;

	adata = dev_get_drvdata(component->dev);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    !adata->mm2s_presence)
		return -ENODEV;
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
		 !adata->s2mm_presence)
		return -ENODEV;

	stream_data = kzalloc(sizeof(*stream_data), GFP_KERNEL);
	if (!stream_data)
		return -ENOMEM;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		ch_count_mask = CFG_MM2S_CH_MASK;
		ch_count_shift = CFG_MM2S_CH_SHIFT;
		data_xfer_mode = CFG_MM2S_XFER_MASK;
		data_xfer_shift = CFG_MM2S_XFER_SHIFT;
		data_format_mode = CFG_MM2S_PKG_MASK;
		stream_data->mmio = adata->mmio + XLNX_MM2S_OFFSET;
		adata->play_stream = substream;

	} else {
		ch_count_mask = CFG_S2MM_CH_MASK;
		ch_count_shift = CFG_S2MM_CH_SHIFT;
		data_xfer_mode = CFG_S2MM_XFER_MASK;
		data_xfer_shift = CFG_S2MM_XFER_SHIFT;
		data_format_mode = CFG_S2MM_PKG_MASK;
		stream_data->mmio = adata->mmio + XLNX_S2MM_OFFSET;
		adata->capture_stream = substream;
	}

	val = ioread32(adata->mmio + XLNX_AUD_CORE_CONFIG);

	if (!(val & data_format_mode))
		stream_data->interleaved = true;
	else
		stream_data->interleaved = false;

	stream_data->xfer_mode = (val & data_xfer_mode) >> data_xfer_shift;
	stream_data->ch_limit = (val & ch_count_mask) >> ch_count_shift;
	dev_info(component->dev,
		 "stream %d : format = %d mode = %d ch_limit = %d\n",
		 substream->stream, stream_data->interleaved,
		 stream_data->xfer_mode, stream_data->ch_limit);

	snd_soc_set_runtime_hwparams(substream, &xlnx_pcm_hardware);
	runtime->private_data = stream_data;

	/* Resize the period bytes as divisible by 64 */
	err = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
					 XLNX_AUD_ALIGN_BYTES);
	if (err) {
		dev_err(component->dev,
			"Unable to set constraint on period bytes\n");
		return err;
	}

	/* Resize the buffer bytes as divisible by 64 */
	err = snd_pcm_hw_constraint_step(runtime, 0,
					 SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
					 XLNX_AUD_ALIGN_BYTES);
	if (err) {
		dev_err(component->dev,
			"Unable to set constraint on buffer bytes\n");
		return err;
	}

	/* Set periods as integer multiple */
	err = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (err < 0) {
		dev_err(component->dev,
			"Unable to set constraint on periods to be integer\n");
		return err;
	}

	/* enable DMA IOC irq */
	val = ioread32(stream_data->mmio + XLNX_AUD_CTRL);
	val |= AUD_CTRL_IOC_IRQ_MASK;
	iowrite32(val, stream_data->mmio + XLNX_AUD_CTRL);

	return 0;
}

static int xlnx_formatter_pcm_close(struct snd_soc_component *component,
				    struct snd_pcm_substream *substream)
{
	int ret;
	struct xlnx_pcm_stream_param *stream_data =
			substream->runtime->private_data;

	if (!component)
		return -ENODEV;

	ret = xlnx_formatter_pcm_reset(stream_data->mmio);
	if (ret) {
		dev_err(component->dev, "audio formatter reset failed\n");
		goto err_reset;
	}
	xlnx_formatter_disable_irqs(stream_data->mmio, substream->stream);

err_reset:
	kfree(stream_data);
	return 0;
}

static snd_pcm_uframes_t
xlnx_formatter_pcm_pointer(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	u32 pos;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xlnx_pcm_stream_param *stream_data = runtime->private_data;

	pos = ioread32(stream_data->mmio + XLNX_AUD_XFER_COUNT);

	if (pos >= stream_data->buffer_size)
		pos = 0;

	return bytes_to_frames(runtime, pos);
}

static int xlnx_formatter_pcm_hw_params(struct snd_soc_component *component,
					struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	u32 low, high, active_ch, val, bits_per_sample, bytes_per_ch;
	u32 aes_reg1_val, aes_reg2_val;
	int status;
	u64 size;
	struct pl_card_data *prv;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct xlnx_pcm_stream_param *stream_data = runtime->private_data;
	struct xlnx_pcm_drv_data *adata;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (!component)
		return -ENODEV;

	adata = dev_get_drvdata(component->dev);

	active_ch = params_channels(params);
	if (active_ch > stream_data->ch_limit)
		return -EINVAL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    adata->sysclk) {
		unsigned int mclk_fs = adata->sysclk / params_rate(params);

		if (adata->sysclk % params_rate(params) != 0) {
			dev_warn(component->dev, "sysclk %u not divisible by rate %u\n",
				 adata->sysclk, params_rate(params));
			return -EINVAL;
		}

		writel(mclk_fs, stream_data->mmio + XLNX_AUD_FS_MULTIPLIER);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
	    stream_data->xfer_mode == AES_TO_PCM &&
	    ((strstr(adata->nodes[XLNX_CAPTURE]->name, "hdmi")) ||
	    (strstr(adata->nodes[XLNX_CAPTURE]->name, "sdi")) ||
	    (strstr(adata->nodes[XLNX_CAPTURE]->name, "dp")))) {
		/*
		 * If formatter is in AES_PCM mode for HDMI/SDI capture path,
		 * parse AES header
		 */
		val = ioread32(stream_data->mmio + XLNX_AUD_STS);
		if (val & AUD_STS_CH_STS_MASK) {
			aes_reg1_val = ioread32(stream_data->mmio +
					 XLNX_AUD_CH_STS_START);
			aes_reg2_val = ioread32(stream_data->mmio +
					 XLNX_AUD_CH_STS_START + 0x4);

			xlnx_parse_aes_params(aes_reg1_val, aes_reg2_val,
					      component->dev);
		}
	}

	size = params_buffer_bytes(params);
	status = snd_pcm_lib_malloc_pages(substream, size);
	if (status < 0)
		return status;

	stream_data->buffer_size = size;

	low = lower_32_bits(runtime->dma_addr);
	high = upper_32_bits(runtime->dma_addr);
	iowrite32(low, stream_data->mmio + XLNX_AUD_BUFF_ADDR_LSB);
	iowrite32(high, stream_data->mmio + XLNX_AUD_BUFF_ADDR_MSB);

	val = ioread32(stream_data->mmio + XLNX_AUD_CTRL);
	val &= ~AUD_CTRL_DATA_WIDTH_MASK;
	bits_per_sample = params_width(params);
	switch (bits_per_sample) {
	case 8:
		val |= (BIT_DEPTH_8 << AUD_CTRL_DATA_WIDTH_SHIFT);
		break;
	case 16:
		val |= (BIT_DEPTH_16 << AUD_CTRL_DATA_WIDTH_SHIFT);
		break;
	case 20:
		val |= (BIT_DEPTH_20 << AUD_CTRL_DATA_WIDTH_SHIFT);
		break;
	case 24:
		val |= (BIT_DEPTH_24 << AUD_CTRL_DATA_WIDTH_SHIFT);
		break;
	case 32:
		val |= (BIT_DEPTH_32 << AUD_CTRL_DATA_WIDTH_SHIFT);
		break;
	}

	val &= ~AUD_CTRL_ACTIVE_CH_MASK;
	val |= active_ch << AUD_CTRL_ACTIVE_CH_SHIFT;
	iowrite32(val, stream_data->mmio + XLNX_AUD_CTRL);

	val = (params_periods(params) << PERIOD_CFG_PERIODS_SHIFT)
		| params_period_bytes(params);
	iowrite32(val, stream_data->mmio + XLNX_AUD_PERIOD_CONFIG);
	bytes_per_ch = DIV_ROUND_UP(params_period_bytes(params), active_ch);
	iowrite32(bytes_per_ch, stream_data->mmio + XLNX_BYTES_PER_CH);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		prv = snd_soc_card_get_drvdata(rtd->card);
		iowrite32(prv->mclk_ratio,
			  stream_data->mmio + XLNX_AUD_FS_MULTIPLIER);
	}

	return 0;
}

static int xlnx_formatter_pcm_hw_free(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int xlnx_formatter_pcm_trigger(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream, int cmd)
{
	u32 val;
	struct xlnx_pcm_stream_param *stream_data =
			substream->runtime->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		val = ioread32(stream_data->mmio + XLNX_AUD_CTRL);
		val |= AUD_CTRL_DMA_EN_MASK;
		iowrite32(val, stream_data->mmio + XLNX_AUD_CTRL);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		val = ioread32(stream_data->mmio + XLNX_AUD_CTRL);
		val &= ~AUD_CTRL_DMA_EN_MASK;
		iowrite32(val, stream_data->mmio + XLNX_AUD_CTRL);
		break;
	}

	return 0;
}

static int xlnx_formatter_pcm_new(struct snd_soc_component *component,
				  struct snd_soc_pcm_runtime *rtd)
{
	if (!component)
		return -ENODEV;

	snd_pcm_lib_preallocate_pages_for_all(rtd->pcm,
					      SNDRV_DMA_TYPE_DEV,
					      component->dev,
					      xlnx_pcm_hardware.buffer_bytes_max,
					      xlnx_pcm_hardware.buffer_bytes_max);
	return 0;
}

static const struct snd_soc_component_driver xlnx_asoc_component = {
	.name		= DRV_NAME,
	.set_sysclk	= xlnx_formatter_set_sysclk,
	.open		= xlnx_formatter_pcm_open,
	.close		= xlnx_formatter_pcm_close,
	.hw_params	= xlnx_formatter_pcm_hw_params,
	.hw_free	= xlnx_formatter_pcm_hw_free,
	.trigger	= xlnx_formatter_pcm_trigger,
	.pointer	= xlnx_formatter_pcm_pointer,
	.pcm_construct	= xlnx_formatter_pcm_new,
};

static int configure_mm2s(struct xlnx_pcm_drv_data *aud_drv_data,
			  struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	aud_drv_data->mm2s_axis_clk = devm_clk_get_enabled(dev, "m_axis_mm2s_aclk");
	if (IS_ERR(aud_drv_data->mm2s_axis_clk)) {
		ret = PTR_ERR(aud_drv_data->mm2s_axis_clk);
		dev_err(dev, "failed to get m_axis_mm2s_aclk(%d)\n", ret);
		return ret;
	}

	aud_drv_data->aud_mclk = devm_clk_get_enabled(dev, "aud_mclk");
	if (IS_ERR(aud_drv_data->aud_mclk)) {
		ret = PTR_ERR(aud_drv_data->aud_mclk);
		dev_err(dev, "failed to get aud_mclk(%d)\n", ret);
		return ret;
	}

	aud_drv_data->mm2s_irq = platform_get_irq_byname(pdev,
							 "irq_mm2s");
	if (aud_drv_data->mm2s_irq < 0)
		return aud_drv_data->mm2s_irq;

	ret = devm_request_irq(dev, aud_drv_data->mm2s_irq,
			       xlnx_mm2s_irq_handler, 0,
			       "xlnx_formatter_pcm_mm2s_irq", dev);
	if (ret) {
		dev_err(dev, "xlnx audio mm2s irq request failed\n");
		return ret;
	}
	ret = xlnx_formatter_pcm_reset(aud_drv_data->mmio +
				       XLNX_MM2S_OFFSET);
	if (ret) {
		dev_err(dev, "audio formatter reset failed\n");
		return ret;
	}
	xlnx_formatter_disable_irqs(aud_drv_data->mmio +
				    XLNX_MM2S_OFFSET,
				    SNDRV_PCM_STREAM_PLAYBACK);

	aud_drv_data->nodes[XLNX_PLAYBACK] =
		of_parse_phandle(dev->of_node, "xlnx,tx", 0);
	if (!aud_drv_data->nodes[XLNX_PLAYBACK])
		dev_err(dev, "tx node not found\n");
	else
		dev_info(dev,
			 "sound card device will use DAI link: %s\n",
			 (aud_drv_data->nodes[XLNX_PLAYBACK])->name);
	of_node_put(aud_drv_data->nodes[XLNX_PLAYBACK]);

	aud_drv_data->mm2s_presence = true;
	return 0;
}

static int configure_s2mm(struct xlnx_pcm_drv_data *aud_drv_data,
			  struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	aud_drv_data->s2mm_axis_clk = devm_clk_get_enabled(dev, "s_axis_s2mm_aclk");
	if (IS_ERR(aud_drv_data->s2mm_axis_clk)) {
		ret = PTR_ERR(aud_drv_data->s2mm_axis_clk);
		dev_err(dev, "failed to get s_axis_s2mm_aclk(%d)\n", ret);
		return ret;
	}

	aud_drv_data->s2mm_irq = platform_get_irq_byname(pdev,
							 "irq_s2mm");
	if (aud_drv_data->s2mm_irq < 0)
		return aud_drv_data->s2mm_irq;

	ret = devm_request_irq(dev, aud_drv_data->s2mm_irq,
			       xlnx_s2mm_irq_handler, 0,
			       "xlnx_formatter_pcm_s2mm_irq",
			       dev);
	if (ret) {
		dev_err(dev, "xlnx audio s2mm irq request failed\n");
		return ret;
	}
	ret = xlnx_formatter_pcm_reset(aud_drv_data->mmio +
				       XLNX_S2MM_OFFSET);
	if (ret) {
		dev_err(dev, "audio formatter reset failed\n");
		return ret;
	}
	xlnx_formatter_disable_irqs(aud_drv_data->mmio +
				    XLNX_S2MM_OFFSET,
				    SNDRV_PCM_STREAM_CAPTURE);

	aud_drv_data->nodes[XLNX_CAPTURE] =
		of_parse_phandle(dev->of_node, "xlnx,rx", 0);
	if (!aud_drv_data->nodes[XLNX_CAPTURE])
		dev_err(dev, "rx node not found\n");
	else
		dev_info(dev, "sound card device will use DAI link: %s\n",
			 (aud_drv_data->nodes[XLNX_CAPTURE])->name);
	of_node_put(aud_drv_data->nodes[XLNX_CAPTURE]);

	aud_drv_data->s2mm_presence = true;
	return 0;
}

static int xlnx_formatter_pcm_probe(struct platform_device *pdev)
{
	int ret;
	u32 val;
	size_t pdata_size;
	struct xlnx_pcm_drv_data *aud_drv_data;
	struct device *dev = &pdev->dev;

	aud_drv_data = devm_kzalloc(dev, sizeof(*aud_drv_data), GFP_KERNEL);
	if (!aud_drv_data)
		return -ENOMEM;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "higher dma address mapping failed %d\n", ret);
		return ret;
	}

	aud_drv_data->axi_clk = devm_clk_get_enabled(dev, "s_axi_lite_aclk");
	if (IS_ERR(aud_drv_data->axi_clk)) {
		ret = PTR_ERR(aud_drv_data->axi_clk);
		dev_err(dev, "failed to get s_axi_lite_aclk(%d)\n", ret);
		return ret;
	}

	aud_drv_data->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(aud_drv_data->mmio)) {
		dev_err(dev, "audio formatter ioremap failed\n");
		return PTR_ERR(aud_drv_data->mmio);
	}

	val = ioread32(aud_drv_data->mmio + XLNX_AUD_CORE_CONFIG);
	if (val & AUD_CFG_MM2S_MASK) {
		ret = configure_mm2s(aud_drv_data, pdev);
		if (ret)
			return ret;
	}

	if (val & AUD_CFG_S2MM_MASK) {
		ret = configure_s2mm(aud_drv_data, pdev);
		if (ret)
			return ret;
	}

	dev_set_drvdata(dev, aud_drv_data);

	ret = devm_snd_soc_register_component(dev, &xlnx_asoc_component,
					      NULL, 0);
	if (ret) {
		dev_err(dev, "pcm platform device register failed\n");
		return ret;
	}

	pdata_size = sizeof(aud_drv_data->nodes);
	if (aud_drv_data->nodes[XLNX_PLAYBACK] ||
	    aud_drv_data->nodes[XLNX_CAPTURE])
		aud_drv_data->pdev =
			platform_device_register_resndata(dev, "xlnx_snd_card",
							  PLATFORM_DEVID_AUTO,
							  NULL, 0,
							  &aud_drv_data->nodes,
							  pdata_size);
	if (!aud_drv_data->pdev)
		dev_err(dev, "sound card device creation failed\n");

	dev_info(dev, "pcm platform device registered\n");
	return 0;
}

static void xlnx_formatter_pcm_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct xlnx_pcm_drv_data *adata = dev_get_drvdata(&pdev->dev);

	platform_device_unregister(adata->pdev);

	if (adata->s2mm_presence)
		ret = xlnx_formatter_pcm_reset(adata->mmio + XLNX_S2MM_OFFSET);

	/* Try MM2S reset, even if S2MM  reset fails */
	if (adata->mm2s_presence)
		ret = xlnx_formatter_pcm_reset(adata->mmio + XLNX_MM2S_OFFSET);

	if (ret)
		dev_err(&pdev->dev, "audio formatter reset failed\n");
}

static const struct of_device_id xlnx_formatter_pcm_of_match[] = {
	{ .compatible = "xlnx,audio-formatter-1.0"},
	{},
};
MODULE_DEVICE_TABLE(of, xlnx_formatter_pcm_of_match);

static struct platform_driver xlnx_formatter_pcm_driver = {
	.probe	= xlnx_formatter_pcm_probe,
	.remove = xlnx_formatter_pcm_remove,
	.driver	= {
		.name	= DRV_NAME,
		.of_match_table	= xlnx_formatter_pcm_of_match,
	},
};

module_platform_driver(xlnx_formatter_pcm_driver);

MODULE_DESCRIPTION("ASoC driver for Xilinx audio formatter");
MODULE_AUTHOR("Maruthi Srinivas Bayyavarapu <maruthis@xilinx.com>");
MODULE_LICENSE("GPL v2");
