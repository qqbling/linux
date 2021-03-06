// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018 Maxime Jourdan <maxi.jourdan@wanadoo.fr>
 */

#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_helpers.h"
#include "canvas.h"

void
codec_helper_set_canvases_yuv420m(struct vdec_session *sess, void *reg_base)
{
	struct vdec_core *core = sess->core;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 64);
	struct v4l2_m2m_buffer *buf;

	/* Setup YUV420 canvases for Decoded Picture Buffer (dpb)
	 * Map them to the user buffers' planes
	 */
	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		u32 buf_idx    = buf->vb.vb2_buf.index;
		u32 cnv_y_idx  = 128 + buf_idx * 3;
		u32 cnv_u_idx = cnv_y_idx + 1;
		u32 cnv_v_idx = cnv_y_idx + 2;
		dma_addr_t buf_y_paddr  =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		dma_addr_t buf_u_paddr =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 1);
		dma_addr_t buf_v_paddr =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 2);

		/* Y plane */
		vdec_canvas_setup(core->dmc_base, cnv_y_idx, buf_y_paddr,
			width, height, MESON_CANVAS_WRAP_NONE,
			MESON_CANVAS_BLKMODE_LINEAR);

		/* U plane */
		vdec_canvas_setup(core->dmc_base, cnv_u_idx, buf_u_paddr,
			width / 2, height / 2, MESON_CANVAS_WRAP_NONE,
			MESON_CANVAS_BLKMODE_LINEAR);

		/* V plane */
		vdec_canvas_setup(core->dmc_base, cnv_v_idx, buf_v_paddr,
			width / 2, height / 2, MESON_CANVAS_WRAP_NONE,
			MESON_CANVAS_BLKMODE_LINEAR);

		writel_relaxed(((cnv_v_idx) << 16) |
			       ((cnv_u_idx) << 8)  |
				(cnv_y_idx), reg_base + buf_idx * 4);
	}
}

void codec_helper_set_canvases_nv12m(struct vdec_session *sess, void *reg_base)
{
	struct vdec_core *core = sess->core;
	u32 width = ALIGN(sess->width, 64);
	u32 height = ALIGN(sess->height, 64);
	struct v4l2_m2m_buffer *buf;

	/* Setup NV12 canvases for Decoded Picture Buffer (dpb)
	 * Map them to the user buffers' planes
	 */
	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		u32 buf_idx    = buf->vb.vb2_buf.index;
		u32 cnv_y_idx  = 128 + buf_idx * 2;
		u32 cnv_uv_idx = cnv_y_idx + 1;
		dma_addr_t buf_y_paddr  =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		dma_addr_t buf_uv_paddr =
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 1);

		/* Y plane */
		vdec_canvas_setup(core->dmc_base, cnv_y_idx, buf_y_paddr,
			width, height, MESON_CANVAS_WRAP_NONE,
			MESON_CANVAS_BLKMODE_LINEAR);

		/* U/V plane */
		vdec_canvas_setup(core->dmc_base, cnv_uv_idx, buf_uv_paddr,
			width, height / 2, MESON_CANVAS_WRAP_NONE,
			MESON_CANVAS_BLKMODE_LINEAR);

		writel_relaxed(((cnv_uv_idx) << 16) |
			       ((cnv_uv_idx) << 8)  |
				(cnv_y_idx), reg_base + buf_idx * 4);
	}
}

void codec_helper_set_canvases(struct vdec_session *sess, void *reg_base)
{
	u32 pixfmt = sess->fmt_cap->pixfmt;

	switch (pixfmt) {
	case V4L2_PIX_FMT_NV12M:
		codec_helper_set_canvases_nv12m(sess, reg_base);
		break;
	case V4L2_PIX_FMT_YUV420M:
		codec_helper_set_canvases_yuv420m(sess, reg_base);
		break;
	default:
		dev_err(sess->core->dev, "Unsupported pixfmt %08X\n", pixfmt);
	};
}