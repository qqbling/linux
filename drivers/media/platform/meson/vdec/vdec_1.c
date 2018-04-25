/*
 * Copyright (C) 2018 Maxime Jourdan <maxi.jourdan@wanadoo.fr>
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
#include <linux/clk.h>

#include "vdec_1.h"

/* AO Registers */
#define AO_RTI_GEN_PWR_SLEEP0	0xe8
#define AO_RTI_GEN_PWR_ISO0	0xec
	#define GEN_PWR_VDEC_1 (BIT(3) | BIT(2))

/* DOS Registers */
#define ASSIST_MBOX1_CLR_REG 0x01d4
#define ASSIST_MBOX1_MASK    0x01d8

#define MPSR 0x0c04
#define CPSR 0x0c84

#define IMEM_DMA_CTRL  0x0d00
#define IMEM_DMA_ADR   0x0d04
#define IMEM_DMA_COUNT 0x0d08
#define LMEM_DMA_CTRL  0x0d40

#define MC_STATUS0  0x2424
#define MC_CTRL1    0x242c

#define DBLK_CTRL   0x2544
#define DBLK_STATUS 0x254c

#define GCLK_EN            0x260c
#define MDEC_PIC_DC_CTRL   0x2638
#define MDEC_PIC_DC_STATUS 0x263c

#define DCAC_DMA_CTRL 0x3848

#define DOS_SW_RESET0             0xfc00
#define DOS_GCLK_EN0              0xfc04
#define DOS_GEN_CTRL0             0xfc08
#define DOS_MEM_PD_VDEC           0xfcc0
#define DOS_VDEC_MCRCC_STALL_CTRL 0xfd00

/* Stream Buffer (stbuf) regs (DOS) */
#define POWER_CTL_VLD 0x3020
#define VLD_MEM_VIFIFO_START_PTR 0x3100
#define VLD_MEM_VIFIFO_CURR_PTR 0x3104
#define VLD_MEM_VIFIFO_END_PTR 0x3108
#define VLD_MEM_VIFIFO_CONTROL 0x3110
	#define MEM_FIFO_CNT_BIT	16
	#define MEM_FILL_ON_LEVEL	BIT(10)
	#define MEM_CTRL_EMPTY_EN	BIT(2)
	#define MEM_CTRL_FILL_EN	BIT(1)
#define VLD_MEM_VIFIFO_WP 0x3114
#define VLD_MEM_VIFIFO_RP 0x3118
#define VLD_MEM_VIFIFO_LEVEL 0x311c
#define VLD_MEM_VIFIFO_BUF_CNTL 0x3120
	#define MEM_BUFCTRL_MANUAL	BIT(1)
#define VLD_MEM_VIFIFO_WRAP_COUNT 0x3144

#define MC_SIZE			(4096 * 4)

static int vdec_1_load_firmware(struct vdec_session *sess, const char* fwname)
{
	const struct firmware *fw;
	struct vdec_core *core = sess->core;
	struct device *dev = core->dev_dec;
	struct vdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;
	static void *mc_addr;
	static dma_addr_t mc_addr_map;
	int ret;
	u32 i = 1000;

	ret = request_firmware(&fw, fwname, dev);
	if (ret < 0)  {
		dev_err(dev, "Unable to request firmware %s\n", fwname);
		return -EINVAL;
	}

	mc_addr = kmalloc(MC_SIZE, GFP_KERNEL);
	if (!mc_addr)
		return -ENOMEM;

	memcpy(mc_addr, fw->data, MC_SIZE);
	mc_addr_map = dma_map_single(core->dev, mc_addr, MC_SIZE, DMA_TO_DEVICE);
	if (!mc_addr_map) {
		dev_err(dev, "Couldn't MAP DMA addr\n");
		return -EINVAL;
	}

	writel_relaxed(0, core->dos_base + MPSR);
	writel_relaxed(0, core->dos_base + CPSR);

	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) & ~(1<<31), core->dos_base + MDEC_PIC_DC_CTRL);

	writel_relaxed(mc_addr_map, core->dos_base + IMEM_DMA_ADR);
	writel_relaxed(MC_SIZE / 4, core->dos_base + IMEM_DMA_COUNT);
	writel_relaxed((0x8000 | (7 << 16)), core->dos_base + IMEM_DMA_CTRL);

	while (--i && readl(core->dos_base + IMEM_DMA_CTRL) & 0x8000) { }

	if (i == 0) {
		printk("Firmware load fail (DMA hang?)\n");
		ret = -EINVAL;
	} else
		printk("Firmware load success\n");

	if (codec_ops->load_extended_firmware)
		codec_ops->load_extended_firmware(sess, fw->data + MC_SIZE, fw->size - MC_SIZE);

	dma_unmap_single(core->dev, mc_addr_map, MC_SIZE, DMA_TO_DEVICE);
	kfree(mc_addr);
	release_firmware(fw);
	return ret;
}

int vdec_1_stbuf_power_up(struct vdec_session *sess) {
	struct vdec_core *core = sess->core;

	writel_relaxed(0, core->dos_base + VLD_MEM_VIFIFO_CONTROL);
	writel_relaxed(0, core->dos_base + VLD_MEM_VIFIFO_WRAP_COUNT);
	writel_relaxed(1 << 4, core->dos_base + POWER_CTL_VLD);

	writel_relaxed(sess->vififo_paddr, core->dos_base + VLD_MEM_VIFIFO_START_PTR);
	writel_relaxed(sess->vififo_paddr, core->dos_base + VLD_MEM_VIFIFO_CURR_PTR);
	writel_relaxed(sess->vififo_paddr + sess->vififo_size - 8, core->dos_base + VLD_MEM_VIFIFO_END_PTR);

	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_CONTROL) |  1, core->dos_base + VLD_MEM_VIFIFO_CONTROL);
	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_CONTROL) & ~1, core->dos_base + VLD_MEM_VIFIFO_CONTROL);

	writel_relaxed(MEM_BUFCTRL_MANUAL, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(sess->vififo_paddr, core->dos_base + VLD_MEM_VIFIFO_WP);

	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL) |  1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL) & ~1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);

	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_CONTROL) | (0x11 << MEM_FIFO_CNT_BIT) | MEM_FILL_ON_LEVEL | MEM_CTRL_FILL_EN | MEM_CTRL_EMPTY_EN, core->dos_base + VLD_MEM_VIFIFO_CONTROL);

	return 0;
}

static void vdec_1_conf_esparser(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;

	/* VDEC_1 specific ESPARSER stuff */
	writel_relaxed(0, core->dos_base + DOS_GEN_CTRL0); // set vififo_vbuf_rp_sel=>vdec
	writel_relaxed(1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL) & ~1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
}

static u32 vdec_1_vififo_level(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;

	return readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_LEVEL);
}

static int vdec_1_start(struct vdec_session *sess)
{
	int ret;
	struct vdec_core *core = sess->core;
	struct vdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	printk("vdec_1_start\n");

	clk_set_rate(core->vdec_1_clk, 666666666);
	ret = clk_prepare_enable(core->vdec_1_clk);
	if (ret)
		return ret;

	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
		GEN_PWR_VDEC_1, 0);
	udelay(10);

	/* Reset VDEC1 */
	writel_relaxed(0xfffffffc, core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0x00000000, core->dos_base + DOS_SW_RESET0);

	writel_relaxed(0x3ff, core->dos_base + DOS_GCLK_EN0);

	/* VDEC Memories */
	writel_relaxed(0x00000000, core->dos_base + DOS_MEM_PD_VDEC);
	/* Remove VDEC1 Isolation */
	regmap_write(core->regmap_ao, AO_RTI_GEN_PWR_ISO0, 0x00000000);
	/* Reset DOS top registers */
	writel_relaxed(0x00000000, core->dos_base + DOS_VDEC_MCRCC_STALL_CTRL);

	writel_relaxed(0x3ff, core->dos_base + GCLK_EN);
	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) & ~(1<<31), core->dos_base + MDEC_PIC_DC_CTRL);

	vdec_1_stbuf_power_up(sess);

	ret = vdec_1_load_firmware(sess, sess->fmt_out->firmware_path);
	if (ret) {
		clk_disable_unprepare(core->vdec_1_clk);
		regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
			GEN_PWR_VDEC_1, GEN_PWR_VDEC_1);
		return ret;
	}

	codec_ops->start(sess);

	/* Enable firmware processor */
	writel_relaxed(1, core->dos_base + MPSR);

	return 0;
}

static int vdec_1_stop(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	printk("vdec_1_stop\n");

	writel_relaxed(0, core->dos_base + MPSR);
	writel_relaxed(0, core->dos_base + CPSR);

	while (readl_relaxed(core->dos_base + IMEM_DMA_CTRL) & 0x8000) { }

	writel_relaxed((1<<12)|(1<<11), core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET0);

	readl_relaxed(core->dos_base + DOS_SW_RESET0);

	writel_relaxed(0, core->dos_base + ASSIST_MBOX1_MASK);

	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) | 1, core->dos_base + MDEC_PIC_DC_CTRL);
	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) & ~1, core->dos_base + MDEC_PIC_DC_CTRL);
	readl_relaxed(core->dos_base + MDEC_PIC_DC_STATUS);

	writel_relaxed(3, core->dos_base + DBLK_CTRL);
	writel_relaxed(0, core->dos_base + DBLK_CTRL);
	readl_relaxed(core->dos_base + DBLK_STATUS);

	writel_relaxed(readl_relaxed(core->dos_base + MC_CTRL1) | 0x9, core->dos_base + MC_CTRL1);
	writel_relaxed(readl_relaxed(core->dos_base + MC_CTRL1) & ~0x9, core->dos_base + MC_CTRL1);
	readl_relaxed(core->dos_base + MC_STATUS0);

	while (readl_relaxed(core->dos_base + DCAC_DMA_CTRL) & 0x8000) { }

	/* enable vdec1 isolation */
	regmap_write(core->regmap_ao, AO_RTI_GEN_PWR_ISO0, 0xc0);
	/* power off vdec1 memories */
	writel(0xffffffffUL, core->dos_base + DOS_MEM_PD_VDEC);
	regmap_update_bits(core->regmap_ao, AO_RTI_GEN_PWR_SLEEP0,
		GEN_PWR_VDEC_1, GEN_PWR_VDEC_1);

	clk_disable_unprepare(core->vdec_1_clk);

	printk("vdec_poweroff end\n");
	return 0;
}

struct vdec_ops vdec_1_ops = {
	.start = vdec_1_start,
	.stop = vdec_1_stop,
	.conf_esparser = vdec_1_conf_esparser,
	.vififo_level = vdec_1_vififo_level,
};