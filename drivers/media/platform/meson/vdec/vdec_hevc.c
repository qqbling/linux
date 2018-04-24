/*
 * Copyright (C) 2018 Maxime Jourdan
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/firmware.h>

#include "vdec_1.h"
#include "hevc_regs.h"

/* AO Registers */
#define AO_RTI_GEN_PWR_ISO0 0xec

/* DOS Registers */
#define ASSIST_MBOX1_CLR_REG 0x01d4
#define ASSIST_MBOX1_MASK    0x01d8

#define DOS_GEN_CTRL0	     0xfc08
#define DOS_SW_RESET3        0xfcd0
#define DOS_MEM_PD_HEVC      0xfccc
#define DOS_GCLK_EN3	     0xfcd4

#define MC_SIZE	(4096 * 4)

static int vdec_hevc_load_firmware(struct vdec_session *sess, const char* fwname)
{
	const struct firmware *fw;
	static void *mc_addr;
	static dma_addr_t mc_addr_map;
	int ret;
	u32 i = 1000;
	struct vdec_core *core = sess->core;
	struct device *dev = core->dev_dec;
	struct vdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	ret = request_firmware(&fw, fwname, dev);
	if (ret < 0)  {
		dev_err(dev, "Unable to request firmware %s\n", fwname);
		return -EINVAL;
	}

	mc_addr = dma_alloc_coherent(core->dev, MC_SIZE, &mc_addr_map, GFP_KERNEL);
	if (!mc_addr) {
		printk("Failed allocating memory for firmware loading\n");
		return -ENOMEM;
	 }

	memcpy(mc_addr, fw->data, MC_SIZE);

	writel_relaxed(0, core->dos_base + HEVC_MPSR);
	writel_relaxed(0, core->dos_base + HEVC_CPSR);

	writel_relaxed(mc_addr_map, core->dos_base + HEVC_IMEM_DMA_ADR);
	writel_relaxed(MC_SIZE / 4, core->dos_base + HEVC_IMEM_DMA_COUNT);
	writel_relaxed((0x8000 | (7 << 16)), core->dos_base + HEVC_IMEM_DMA_CTRL);

	while (--i && readl(core->dos_base + HEVC_IMEM_DMA_CTRL) & 0x8000) { }

	if (i == 0) {
		printk("Firmware load fail (DMA hang?)\n");
		ret = -EINVAL;
	} else
		printk("Firmware load success\n");

	if (codec_ops->load_extended_firmware)
		codec_ops->load_extended_firmware(sess, fw->data + MC_SIZE, fw->size - MC_SIZE);

	dma_free_coherent(core->dev, MC_SIZE, mc_addr, mc_addr_map);
	release_firmware(fw);
	return ret;
}

static void vdec_hevc_stbuf_init(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;

	printk("vdec_hevc_stbuf_init\n");

	writel_relaxed(readl_relaxed(core->dos_base + HEVC_STREAM_CONTROL) & ~1, core->dos_base + HEVC_STREAM_CONTROL);
	writel_relaxed(sess->vififo_paddr, core->dos_base + HEVC_STREAM_START_ADDR);
	writel_relaxed(sess->vififo_paddr + sess->vififo_size, core->dos_base + HEVC_STREAM_END_ADDR);
	writel_relaxed(sess->vififo_paddr, core->dos_base + HEVC_STREAM_RD_PTR);
	writel_relaxed(sess->vififo_paddr, core->dos_base + HEVC_STREAM_WR_PTR);

	printk("vdec_hevc_stbuf_init end\n");
}

/* VDEC_HEVC specific ESPARSER configuration */
static void vdec_hevc_conf_esparser(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;

	printk("vdec_hevc_conf_esparser\n");

	/* set vififo_vbuf_rp_sel=>vdec_hevc */
	writel_relaxed(3 << 1, core->dos_base + DOS_GEN_CTRL0);
	writel_relaxed(readl_relaxed(core->dos_base + HEVC_STREAM_CONTROL) | (1 << 3), core->dos_base + HEVC_STREAM_CONTROL);
	writel_relaxed(readl_relaxed(core->dos_base + HEVC_STREAM_CONTROL) | 1, core->dos_base + HEVC_STREAM_CONTROL);
	writel_relaxed(readl_relaxed(core->dos_base + HEVC_STREAM_FIFO_CTL) | (1 << 29), core->dos_base + HEVC_STREAM_FIFO_CTL);
}

static u32 vdec_hevc_vififo_level(struct vdec_session *sess)
{
	/* TODO */
	return 0;
}

static int vdec_hevc_start(struct vdec_session *sess)
{
	int ret;
	struct vdec_core *core = sess->core;
	struct vdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	printk("vdec_hevc_start\n");

	/* Reset VDEC_HEVC*/
	writel_relaxed(0xffffffff, core->dos_base + DOS_SW_RESET3);
	writel_relaxed(0x00000000, core->dos_base + DOS_SW_RESET3);

	writel_relaxed(0xffffffff, core->dos_base + DOS_GCLK_EN3);

	/* VDEC_HEVC Memories */
	writel_relaxed(0x00000000, core->dos_base + DOS_MEM_PD_HEVC);

	/* Remove VDEC_HEVC Isolation */
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_ISO0, 0xc00, 0);

	writel_relaxed(0xffffffff, core->dos_base + DOS_SW_RESET3);
	udelay(10);
	writel_relaxed(0x00000000, core->dos_base + DOS_SW_RESET3);

	vdec_hevc_stbuf_init(sess);

	ret = vdec_hevc_load_firmware(sess, sess->fmt_out->firmware_path);
	if (ret)
		return ret;

	codec_ops->start(sess);

	writel_relaxed((1<<12)|(1<<11), core->dos_base + DOS_SW_RESET3);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET3);
	readl_relaxed(core->dos_base + DOS_SW_RESET3);

	writel_relaxed(1, core->dos_base + HEVC_MPSR);

	printk("vdec_hevc_start end\n");

	return 0;
}

static int vdec_hevc_stop(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	printk("vdec_hevc_stop\n");

	/* Enable VDEC_HEVC Isolation */
	regmap_write(core->regmap_ao, AO_RTI_GEN_PWR_ISO0, 0xc00);

	/* VDEC_HEVC Memories */
	writel_relaxed(0xffffffffUL, core->dos_base + DOS_MEM_PD_HEVC);

	return 0;
}

struct vdec_ops vdec_hevc_ops = {
	.start = vdec_hevc_start,
	.stop = vdec_hevc_stop,
	.conf_esparser = vdec_hevc_conf_esparser,
	.vififo_level = vdec_hevc_vififo_level,
};