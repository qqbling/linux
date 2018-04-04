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
	#define PS_CFG_MAX_FETCH_CYCLE_BIT  0
	#define PS_CFG_STARTCODE_WID_24_BIT 10
	#define PS_CFG_MAX_ES_WR_CYCLE_BIT  12
	#define PS_CFG_PFIFO_EMPTY_CNT_BIT  16
#define PARSER_CONTROL 0x00
	#define ES_PACK_SIZE_BIT	8
	#define ES_WRITE		BIT(5)
	#define ES_SEARCH		BIT(1)
	#define ES_PARSER_START		BIT(0)
#define PFIFO_RD_PTR 0x1c
#define PFIFO_WR_PTR 0x18
#define PARSER_SEARCH_PATTERN 0x24
	#define ES_START_CODE_PATTERN 0x00000100
#define PARSER_SEARCH_MASK 0x28
	#define ES_START_CODE_MASK	0xffffff00
#define PARSER_FETCH_ADDR 0x4
#define PARSER_FETCH_CMD  0x8
	#define FETCH_ENDIAN_BIT	  27

/* STBUF regs */
#define VLD_MEM_VIFIFO_BUF_CNTL 0x3120
	#define MEM_BUFCTRL_MANUAL	BIT(1)

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

/**
 * Userspace is very likely to feed us packets with timestamps not in chronological order
 * because of B-frames. Rearrange them here.
 */
static void add_buffer_to_list(struct vdec_session *sess, struct vdec_buffer *new_buf) {
	struct vdec_buffer *tmp;
	unsigned long flags;

	spin_lock_irqsave(&sess->bufs_spinlock, flags);
	if (list_empty(&sess->bufs))
		goto add_core;

	list_for_each_entry(tmp, &sess->bufs, list) {
		if (new_buf->timestamp < tmp->timestamp) {
			list_add_tail(&new_buf->list, &tmp->list);
			goto unlock;
		}
	}

add_core:
	list_add_tail(&new_buf->list, &sess->bufs);
unlock:
	spin_unlock_irqrestore(&sess->bufs_spinlock, flags);
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

int esparser_process_buf(struct vdec_core *core, struct vb2_v4l2_buffer *vbuf) {
	struct vb2_buffer *vb = &vbuf->vb2_buf;
	dma_addr_t phy = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);
	u32 payload_size = vb2_get_plane_payload(vb, 0);

	esparser_append_start_code(vb);

	writel_relaxed(0, core->esparser_base + PFIFO_RD_PTR);
	writel_relaxed(0, core->esparser_base + PFIFO_WR_PTR);
	writel_relaxed(ES_WRITE | ES_PARSER_START | ES_SEARCH | (payload_size << ES_PACK_SIZE_BIT), core->esparser_base + PARSER_CONTROL);

	writel_relaxed(phy, core->esparser_base + PARSER_FETCH_ADDR);
	writel_relaxed((7 << FETCH_ENDIAN_BIT) | (payload_size + 512), core->esparser_base + PARSER_FETCH_CMD);
	search_done = 0;

	return wait_event_interruptible_timeout(wq, search_done != 0, HZ/5);
}

int esparser_queue(void *data) {
	struct vdec_session *sess = data;
	struct vdec_core *core = sess->core;
	struct v4l2_m2m_buffer *buf, *n;
	struct vdec_buffer *new_buf;
	int ret;

	for (;;) {
		ret = wait_event_interruptible(sess->input_buf_wq, sess->input_bufs_ready == 1 || kthread_should_stop());
		if (kthread_should_stop())
			break;

		if (ret == -EINTR)
			continue;

		sess->input_bufs_ready = 0;

		v4l2_m2m_for_each_src_buf_safe(sess->m2m_ctx, buf, n) {
			struct vb2_v4l2_buffer *vbuf = &buf->vb;
			v4l2_m2m_src_buf_remove_by_buf(sess->m2m_ctx, vbuf);

			while (down_timeout(&sess->queue_sema, HZ) < 0) {
				if (kthread_should_stop()) {
					v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
					goto end;
				}

				printk("Timed out waiting for an input slot. Trying again..\n");
			}

			ret = esparser_process_buf(core, vbuf);

			if (ret > 0) {
				struct vb2_buffer *vb = &vbuf->vb2_buf;
				new_buf = kmalloc(sizeof(struct vdec_buffer), GFP_KERNEL);
				new_buf->timestamp = vb->timestamp;
				new_buf->index = -1;
				add_buffer_to_list(sess, new_buf);

				vbuf->flags = 0;
				vbuf->field = V4L2_FIELD_NONE;
				v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
			} else if (ret <= 0) {
				printk("ESPARSER input parsing fatal error\n");
				v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
				writel_relaxed(0, core->esparser_base + PARSER_FETCH_CMD);
				up(&sess->queue_sema);
			}
		}
	}

end:
	return 0;
}

int esparser_power_up(struct vdec_session *sess) {
	struct vdec_core *core = sess->core;

	// WRITE_MPEG_REG(FEC_INPUT_CONTROL, 0);
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

	/* parser video */
	writel_relaxed(sess->vififo_paddr, core->esparser_base + PARSER_VIDEO_START_PTR);
	writel_relaxed(sess->vififo_paddr + sess->vififo_size, core->esparser_base + PARSER_VIDEO_END_PTR);
	writel_relaxed(readl_relaxed(core->esparser_base + PARSER_ES_CONTROL) & ~1, core->esparser_base + PARSER_ES_CONTROL);
	
	writel_relaxed(0xffff, core->esparser_base + PARSER_INT_STATUS);
	writel_relaxed(1 << PARSER_INT_HOST_EN_BIT, core->esparser_base + PARSER_INT_ENABLE);

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
	/*core->fake_pattern = (unsigned char *)kcalloc(1, SEARCH_PATTERN_LEN, GFP_KERNEL);
	core->fake_pattern[0] = 0x00;
	core->fake_pattern[1] = 0x00;
	core->fake_pattern[2] = 0x01;
	core->fake_pattern[3] = 0xff;
	core->fake_pattern_map = dma_map_single(NULL, core->fake_pattern,
						SEARCH_PATTERN_LEN, DMA_TO_DEVICE);*/

	return 0;
}