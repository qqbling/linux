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

#ifndef __MESON_VDEC_CORE_H_
#define __MESON_VDEC_CORE_H_

#include <linux/regmap.h>
#include <linux/list.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define REG_BUF_SIZE 21

struct vdec_buffer {
	struct vb2_v4l2_buffer vb;
};

struct reg_buffer {
	u32 flags;
	u64 timestamp;
};

struct vdec_core {
	void __iomem *dos_base;
	void __iomem *esparser_base;
	void __iomem *dmc_base;
	struct regmap *regmap_ao;
    int irq;
	struct device *dev;
	struct device *dev_dec;

	struct video_device *vdev_dec;
	struct v4l2_device v4l2_dev;
	struct v4l2_fh fh;
	struct v4l2_m2m_dev *m2m_dev;
	struct v4l2_m2m_ctx *m2m_ctx;
	
	struct mutex lock;
	
	struct reg_buffer reg_buffers[REG_BUF_SIZE];
	int reg_buf_start;
	int reg_buf_end;
	
	/* Big contiguous area for the Decoded Picture Buffer */
	void *vbuf_vaddr;
	dma_addr_t vbuf_paddr;
	
	/* Fake Start Code for the ESPARSER to trigger the IRQs */
	unsigned char *fake_pattern;
	dma_addr_t     fake_pattern_map;
	
	/* H.264 decoder requires an extended firmware loaded in contiguous RAM */
	void      *vh264_ext_fw_vaddr;
	dma_addr_t vh264_ext_fw_paddr;
	
	/* The decoder requires a "post canvas", don't really know what it's for */
	void      *dummy_post_canvas_vaddr;
	dma_addr_t dummy_post_canvas_paddr;
	
	unsigned int streamon_cap, streamon_out;
	
	u32 colorspace;
	u8 ycbcr_enc;
	u8 quantization;
	u8 xfer_func;
};

#endif
