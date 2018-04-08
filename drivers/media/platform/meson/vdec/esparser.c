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
#define PARSER_CONTROL 0x00
	#define ES_PACK_SIZE_BIT	8
	#define ES_WRITE		BIT(5)
	#define ES_SEARCH		BIT(1)
	#define ES_PARSER_START		BIT(0)
#define PARSER_FETCH_ADDR 0x4
#define PARSER_FETCH_CMD  0x8
#define PARSER_CONFIG 0x14
	#define PS_CFG_MAX_FETCH_CYCLE_BIT  0
	#define PS_CFG_STARTCODE_WID_24_BIT 10
	#define PS_CFG_MAX_ES_WR_CYCLE_BIT  12
	#define PS_CFG_PFIFO_EMPTY_CNT_BIT  16
#define PFIFO_WR_PTR 0x18
#define PFIFO_RD_PTR 0x1c
#define PARSER_SEARCH_PATTERN 0x24
	#define ES_START_CODE_PATTERN 0x00000100
#define PARSER_SEARCH_MASK 0x28
	#define ES_START_CODE_MASK	0xffffff00
	#define FETCH_ENDIAN_BIT	  27
#define PARSER_INT_ENABLE 0x2c
	#define PARSER_INT_HOST_EN_BIT 8
#define PARSER_INT_STATUS 0x30
	#define PARSER_INTSTAT_SC_FOUND 1
#define PARSER_ES_CONTROL 0x5c
#define PARSER_VIDEO_START_PTR 0x80
#define PARSER_VIDEO_END_PTR 0x84
#define PARSER_VIDEO_HOLE 0x90

/* STBUF regs */
#define VLD_MEM_VIFIFO_BUF_CNTL 0x3120
	#define MEM_BUFCTRL_MANUAL	BIT(1)

#define SEARCH_PATTERN_LEN   512

static DECLARE_WAIT_QUEUE_HEAD(wq);
static int search_done;

static irqreturn_t esparser_isr(int irq, void *dev)
{
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

/* Add a start code at the end of the buffer
 * to trigger the esparser interrupt
 */
static void esparser_append_start_code(struct vb2_buffer *vb)
{
	u8 *vaddr = vb2_plane_vaddr(vb, 0) + vb2_get_plane_payload(vb, 0);

	vaddr[0] = 0x00;
	vaddr[1] = 0x00;
	vaddr[2] = 0x01;
	vaddr[3] = 0xff;
}

static int esparser_process_buf(struct vdec_core *core, struct vb2_buffer *vb)
{
	dma_addr_t phy = vb2_dma_contig_plane_dma_addr(vb, 0);
	u32 payload_size = vb2_get_plane_payload(vb, 0);

	esparser_append_start_code(vb);

	/* Throwing in buffers too quickly (100+ fps) will unfortunately result
	 * in random decode errors. There doesn't seem to be any way
	 * to predict this.
	 * This delay still permits ~100fps decoding
	 */
	//usleep_range(5000, 10000);
	writel_relaxed(0, core->esparser_base + PFIFO_RD_PTR);
	writel_relaxed(0, core->esparser_base + PFIFO_WR_PTR);
	writel_relaxed(ES_WRITE | ES_PARSER_START | ES_SEARCH | (payload_size << ES_PACK_SIZE_BIT), core->esparser_base + PARSER_CONTROL);

	writel_relaxed(phy, core->esparser_base + PARSER_FETCH_ADDR);
	writel_relaxed((7 << FETCH_ENDIAN_BIT) | (payload_size + 512), core->esparser_base + PARSER_FETCH_CMD);
	search_done = 0;

	return wait_event_interruptible_timeout(wq, search_done != 0, HZ/5);
}

static u32 esparser_vififo_free_space(struct vdec_session *sess)
{
	u32 vififo_usage;
	struct vdec_ops *vdec_ops = sess->fmt_out->vdec_ops;
	struct vdec_core *core = sess->core;

	vififo_usage  = vdec_ops->vififo_level(sess);
	vififo_usage += readl_relaxed(core->esparser_base + PARSER_VIDEO_HOLE);
	vififo_usage += (6 * SZ_1K);

	if (vififo_usage > sess->vififo_size) {
		dev_warn(sess->core->dev_dec,
			"VIFIFO usage (%u) > VIFIFO size (%u)\n",
			vififo_usage, sess->vififo_size);
		return 0;
	}

	return sess->vififo_size - vififo_usage;
}

static int esparser_queue(struct vdec_session *sess, struct vb2_v4l2_buffer *vbuf)
{
	int ret;
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	struct vdec_core *core = sess->core;
	u32 payload_size = vb2_get_plane_payload(vb, 0);

	if (esparser_vififo_free_space(sess) < payload_size ||
	    atomic_read(&sess->esparser_queued_bufs) >= 17)
		return -EAGAIN;

	v4l2_m2m_src_buf_remove_by_buf(sess->m2m_ctx, vbuf);
	vdec_add_buf_reorder(sess, vb->timestamp);

	ret = esparser_process_buf(core, vb);

	if (ret > 0) {
		vbuf->flags = 0;
		vbuf->field = V4L2_FIELD_NONE;
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
	} else if (ret <= 0) {
		printk("ESPARSER input parsing error\n");
		vdec_remove_buf(sess, vb->timestamp);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		writel_relaxed(0, core->esparser_base + PARSER_FETCH_CMD);
	}

	return 0;
}

void esparser_queue_all_src(struct work_struct *work)
{
	struct v4l2_m2m_buffer *buf, *n;
	struct vdec_session *sess =
		container_of(work, struct vdec_session, esparser_queue_work);

	v4l2_m2m_for_each_src_buf_safe(sess->m2m_ctx, buf, n) {
		if (esparser_queue(sess, &buf->vb) < 0)
			break;

		atomic_inc(&sess->esparser_queued_bufs);
	}
}

int esparser_power_up(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	struct vdec_ops *vdec_ops = sess->fmt_out->vdec_ops;

	writel_relaxed((10 << PS_CFG_PFIFO_EMPTY_CNT_BIT) |
				(1  << PS_CFG_MAX_ES_WR_CYCLE_BIT) |
				(16 << PS_CFG_MAX_FETCH_CYCLE_BIT),
				core->esparser_base + PARSER_CONFIG);

	writel_relaxed(0, core->esparser_base + PFIFO_RD_PTR);
	writel_relaxed(0, core->esparser_base + PFIFO_WR_PTR);

	writel_relaxed(ES_START_CODE_PATTERN, core->esparser_base + PARSER_SEARCH_PATTERN);
	writel_relaxed(ES_START_CODE_MASK,    core->esparser_base + PARSER_SEARCH_MASK);

	writel_relaxed((10 << PS_CFG_PFIFO_EMPTY_CNT_BIT) |
				   (1  << PS_CFG_MAX_ES_WR_CYCLE_BIT) |
				   (16 << PS_CFG_MAX_FETCH_CYCLE_BIT) |
				   (2  << PS_CFG_STARTCODE_WID_24_BIT),
				   core->esparser_base + PARSER_CONFIG);

	writel_relaxed((ES_SEARCH | ES_PARSER_START), core->esparser_base + PARSER_CONTROL);

	writel_relaxed(sess->vififo_paddr, core->esparser_base + PARSER_VIDEO_START_PTR);
	writel_relaxed(sess->vififo_paddr + sess->vififo_size - 8, core->esparser_base + PARSER_VIDEO_END_PTR);
	writel_relaxed(readl_relaxed(core->esparser_base + PARSER_ES_CONTROL) & ~1, core->esparser_base + PARSER_ES_CONTROL);
	
	if (vdec_ops->conf_esparser)
		vdec_ops->conf_esparser(sess);

	writel_relaxed(0xffff, core->esparser_base + PARSER_INT_STATUS);
	writel_relaxed(1 << PARSER_INT_HOST_EN_BIT, core->esparser_base + PARSER_INT_ENABLE);

	return 0;
}

int esparser_init(struct platform_device *pdev, struct vdec_core *core)
{
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
	/*core->fake_pattern = (unsigned char *)kcalloc(1, SEARCH_PATTERN_LEN, GFP_KERNEL);
	core->fake_pattern[0] = 0x00;
	core->fake_pattern[1] = 0x00;
	core->fake_pattern[2] = 0x01;
	core->fake_pattern[3] = 0xff;
	core->fake_pattern_map = dma_map_single(NULL, core->fake_pattern,
						SEARCH_PATTERN_LEN, DMA_TO_DEVICE);*/

	return 0;
}