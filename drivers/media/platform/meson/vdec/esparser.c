/*
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

#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mem2mem.h>

#include "esparser.h"

/* PARSER REGS (CBUS) */
#define PARSER_INT_STATUS 0x30
	#define PARSER_INTSTAT_SC_FOUND 1
#define PARSER_INT_ENABLE 0x2c
	#define PARSER_INT_HOST_EN_BIT 8
#define PARSER_VIDEO_START_PTR 0x80
#define PARSER_VIDEO_END_PTR 0x84
#define PARSER_ES_CONTROL 0x5c
#define PARSER_CONFIG 0x14
	#define PS_CFG_MAX_FETCH_CYCLE_BIT 0
	#define PS_CFG_STARTCODE_WID_24	10
	#define PS_CFG_MAX_ES_WR_CYCLE_BIT 12
	#define PS_CFG_PFIFO_EMPTY_CNT_BIT 16
#define PARSER_CONTROL 0x00
	#define ES_PACK_SIZE_BIT				8
	#define ES_WRITE						(1<<5)
	#define ES_SEARCH					   (1<<1)
	#define ES_PARSER_START				 (1<<0)
#define PFIFO_RD_PTR 0x1c
#define PFIFO_WR_PTR 0x18
#define PARSER_SEARCH_PATTERN 0x24
	#define ES_START_CODE_PATTERN 0x00000100
#define PARSER_SEARCH_MASK 0x28
	#define ES_START_CODE_MASK	0xffffff00
#define PARSER_FETCH_ADDR 0x4
#define PARSER_FETCH_CMD  0x8
	#define FETCH_ENDIAN_BIT	  27

/* DOS REGS */
#define DOS_GEN_CTRL0 0xfc08

/* Stream Buffer (stbuf) regs (DOS) */
#define POWER_CTL_VLD 0x3020
#define VLD_MEM_VIFIFO_START_PTR 0x3100
#define VLD_MEM_VIFIFO_CURR_PTR 0x3104
#define VLD_MEM_VIFIFO_END_PTR 0x3108
#define VLD_MEM_VIFIFO_CONTROL 0x3110
	#define MEM_BUFCTRL_MANUAL		(1<<1)
	#define MEM_BUFCTRL_INIT		(1<<0)
	#define MEM_LEVEL_CNT_BIT	   18
	#define MEM_FIFO_CNT_BIT		16
	#define MEM_FILL_ON_LEVEL		(1<<10)
	#define MEM_CTRL_EMPTY_EN		(1<<2)
	#define MEM_CTRL_FILL_EN		(1<<1)
	#define MEM_CTRL_INIT			(1<<0)
#define VLD_MEM_VIFIFO_WP 0x3114
#define VLD_MEM_VIFIFO_RP 0x3118
#define VLD_MEM_VIFIFO_BUF_CNTL 0x3120
#define VLD_MEM_VIFIFO_WRAP_COUNT 0x3144

#define SEARCH_PATTERN_LEN   512

static DECLARE_WAIT_QUEUE_HEAD(wq);
static int search_done;

static irqreturn_t esparser_isr(int irq, void *dev) {
	int int_status;
	struct vdec_core *core = dev;

	int_status = readl_relaxed(core->esparser_base + PARSER_INT_STATUS);
	writel_relaxed(int_status, core->esparser_base + PARSER_INT_STATUS);

	//printk("esparser_isr! status = %08X\n", int_status);

	if (int_status & PARSER_INTSTAT_SC_FOUND) {
		writel_relaxed(0, core->esparser_base + PFIFO_RD_PTR);
		writel_relaxed(0, core->esparser_base + PFIFO_WR_PTR);
		search_done = 1;
		wake_up_interruptible(&wq);
	}

	return IRQ_HANDLED;
}

static int first_pkt = 1;

/**
 * Userspace is very likely to feed us packets with timestamps not in chronological order
 * because of B-frames. Rearrange them here.
 */
static void add_buffer_to_list(struct vdec_core *core, struct vdec_buffer *new_buf) {
	struct vdec_buffer *tmp;

	spin_lock(&core->bufs_spinlock);
	if (list_empty(&core->bufs))
		goto add_core;

	list_for_each_entry(tmp, &core->bufs, list) {
		if (new_buf->timestamp < tmp->timestamp) {
			list_add_tail(&new_buf->list, &tmp->list);
			goto unlock;
		}
	}

add_core:
	list_add_tail(&new_buf->list, &core->bufs);
unlock:
	spin_unlock(&core->bufs_spinlock);
}

int esparser_process_buf(struct vdec_core *core, struct vb2_v4l2_buffer *vbuf) {
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	struct vdec_buffer *new_buf;
	int ret;
	dma_addr_t phy = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);

	//printk("Putting buffer with address %08X; len %d\n", phy, vb2_get_plane_payload(vb, 0));
	down(&core->queue_sema);
	wmb();
	writel_relaxed(0, core->esparser_base + PFIFO_RD_PTR);
	writel_relaxed(0, core->esparser_base + PFIFO_WR_PTR);
	writel_relaxed(ES_WRITE | ES_PARSER_START | ES_SEARCH | ((vb2_get_plane_payload(vb, 0) << ES_PACK_SIZE_BIT)), core->esparser_base + PARSER_CONTROL);

	writel_relaxed(phy, core->esparser_base + PARSER_FETCH_ADDR);
	writel_relaxed((7 << FETCH_ENDIAN_BIT) | vb2_get_plane_payload(vb, 0), core->esparser_base + PARSER_FETCH_CMD);
	search_done = 0;
	writel_relaxed(core->fake_pattern_map, core->esparser_base + PARSER_FETCH_ADDR);
	writel_relaxed((7 << FETCH_ENDIAN_BIT) | SEARCH_PATTERN_LEN, core->esparser_base + PARSER_FETCH_CMD);

	ret = wait_event_interruptible_timeout(wq, search_done != 0, HZ/5);

	v4l2_m2m_src_buf_remove_by_buf(core->m2m_ctx, vbuf);
	if (ret > 0) {
		//msleep(30); // Don't go too fast.. Very hacky for now
		schedule_work(&core->mark_buffers_done_work);
		new_buf = kmalloc(sizeof(struct vdec_buffer), GFP_KERNEL);
		new_buf->timestamp = vb->timestamp;
		new_buf->index = -1;

		add_buffer_to_list(core, new_buf);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
	} else if (ret <= 0) {
		printk("Write timeout\n");
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		writel_relaxed(0, core->esparser_base + PARSER_FETCH_CMD);
	}

	return 0;
}

int esparser_power_up(struct vdec_core *core) {
	// WRITE_MPEG_REG(FEC_INPUT_CONTROL, 0);
	writel_relaxed((10 << PS_CFG_PFIFO_EMPTY_CNT_BIT) |
				   (1  << PS_CFG_MAX_ES_WR_CYCLE_BIT)|
				   (16 << PS_CFG_MAX_FETCH_CYCLE_BIT),
				   core->esparser_base + PARSER_CONFIG);

	writel_relaxed(0, core->esparser_base + PFIFO_RD_PTR);
	writel_relaxed(0, core->esparser_base + PFIFO_WR_PTR);

	writel_relaxed(ES_START_CODE_PATTERN, core->esparser_base + PARSER_SEARCH_PATTERN);
	writel_relaxed(ES_START_CODE_MASK,	core->esparser_base + PARSER_SEARCH_MASK);

	writel_relaxed((10 << PS_CFG_PFIFO_EMPTY_CNT_BIT) |
				   (1  << PS_CFG_MAX_ES_WR_CYCLE_BIT) |
				   (16 << PS_CFG_MAX_FETCH_CYCLE_BIT) |
				   (2  << PS_CFG_STARTCODE_WID_24),
				   core->esparser_base + PARSER_CONFIG);

	writel_relaxed((ES_SEARCH | ES_PARSER_START), core->esparser_base + PARSER_CONTROL);

	/* parser video */
	writel_relaxed(core->vififo_paddr, core->esparser_base + PARSER_VIDEO_START_PTR);
	writel_relaxed(core->vififo_paddr + core->vififo_size, core->esparser_base + PARSER_VIDEO_END_PTR);
	writel_relaxed(1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL) & ~1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(0, core->dos_base + DOS_GEN_CTRL0); // set vififo_vbuf_rp_sel=>vdec
	
	writel_relaxed(0xffff, core->esparser_base + PARSER_INT_STATUS);
	writel_relaxed(1 << PARSER_INT_HOST_EN_BIT, core->esparser_base + PARSER_INT_ENABLE);

	return 0;
}

/* Is this actually necessary? */
int stbuf_power_up(struct vdec_core *core) {
	writel_relaxed(0, core->dos_base + VLD_MEM_VIFIFO_CONTROL);
	writel_relaxed(0, core->dos_base + VLD_MEM_VIFIFO_WRAP_COUNT);
	writel_relaxed(1 << 4, core->dos_base + POWER_CTL_VLD);

	writel_relaxed(core->vififo_paddr, core->dos_base + VLD_MEM_VIFIFO_START_PTR);
	writel_relaxed(core->vififo_paddr, core->dos_base + VLD_MEM_VIFIFO_CURR_PTR);
	writel_relaxed(core->vififo_paddr + core->vififo_size - 8, core->dos_base + VLD_MEM_VIFIFO_END_PTR);

	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_CONTROL) |  1, core->dos_base + VLD_MEM_VIFIFO_CONTROL);
	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_CONTROL) & ~1, core->dos_base + VLD_MEM_VIFIFO_CONTROL);

	writel_relaxed(MEM_BUFCTRL_MANUAL, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(core->vififo_paddr, core->dos_base + VLD_MEM_VIFIFO_WP);

	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL) |  1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);
	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL) & ~1, core->dos_base + VLD_MEM_VIFIFO_BUF_CNTL);

	writel_relaxed(readl_relaxed(core->dos_base + VLD_MEM_VIFIFO_CONTROL) | (0x11 << 16) | MEM_FILL_ON_LEVEL | MEM_CTRL_FILL_EN | MEM_CTRL_EMPTY_EN, core->dos_base + VLD_MEM_VIFIFO_CONTROL);

	return 0;
}

int esparser_init(struct platform_device *pdev, struct vdec_core *core) {
	int ret;
	int irq;

	/* TODO: name the IRQs */
	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		printk("Failed getting IRQ\n");
		return irq;
	}

	printk("Requesting IRQ %d\n", irq);

	ret = devm_request_irq(&pdev->dev, irq, esparser_isr,
					IRQF_SHARED,
					"esparserirq", core);
	if (ret) {
		printk("Failed requesting IRQ\n");
		return ret;
	}

	/* Generate a fake start code to trigger the esparser IRQ later on */
	core->fake_pattern = (unsigned char *)kcalloc(1, SEARCH_PATTERN_LEN, GFP_KERNEL);
	core->fake_pattern[0] = 0x00;
	core->fake_pattern[1] = 0x00;
	core->fake_pattern[2] = 0x01;
	core->fake_pattern[3] = 0xff;
	core->fake_pattern_map = dma_map_single(NULL, core->fake_pattern,
						SEARCH_PATTERN_LEN, DMA_TO_DEVICE);

	return 0;
}