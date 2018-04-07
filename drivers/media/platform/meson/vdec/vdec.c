#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/syscon.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>

#include "vdec.h"
#include "esparser.h"
#include "canvas.h"

#include "vdec_1.h"
#include "vdec_hevc.h"

#include "codec_mpeg12.h"
#include "codec_mpeg4.h"
#include "codec_h264.h"
#include "codec_hevc.h"

static void vdec_abort(struct vdec_session *sess) {
	printk("Aborting decoding session!\n");
	vb2_queue_error(&sess->m2m_ctx->cap_q_ctx.q);
	vb2_queue_error(&sess->m2m_ctx->out_q_ctx.q);
}

static u32 get_output_size(u32 width, u32 height) {
	return ALIGN(width, 64) * ALIGN(height, 64);
}

u32 vdec_get_output_size(struct vdec_session *sess) {
	return get_output_size(sess->width, sess->height);
}

static int vdec_poweron(struct vdec_session *sess) {
	int ret;
	struct vdec_ops *vdec_ops = sess->fmt_out->vdec_ops;

	printk("vdec_poweron\n");

	ret = vdec_ops->start(sess);
	if (ret)
		return ret;

	esparser_power_up(sess);

	return 0;
}

static void vdec_poweroff(struct vdec_session *sess) {
	struct vdec_ops *vdec_ops = sess->fmt_out->vdec_ops;
	struct vdec_codec_ops *codec_ops = sess->fmt_out->codec_ops;

	kthread_stop(sess->esparser_queue_thread);

	codec_ops->stop(sess);
	vdec_ops->stop(sess);
}

void vdec_m2m_device_run(void *priv) {
	struct vdec_session *sess = priv;

	printk("vdec_m2m_device_run\n");
	mutex_lock(&sess->lock);

	sess->input_bufs_ready = 1;
	wake_up_interruptible(&sess->input_buf_wq);

	mutex_unlock(&sess->lock);
}

void vdec_m2m_job_abort(void *priv) {
	struct vdec_session *sess = priv;

	printk("vdec_m2m_job_abort\n");
	v4l2_m2m_job_finish(sess->m2m_dev, sess->m2m_ctx);
}

static const struct v4l2_m2m_ops vdec_m2m_ops = {
	.device_run = vdec_m2m_device_run,
	.job_abort = vdec_m2m_job_abort,
};

static int vdec_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct vdec_session *sess = vb2_get_drv_priv(q);
	struct vdec_format *fmt_out = sess->fmt_out;
	struct vdec_format *fmt_cap = sess->fmt_cap;
	printk("vdec_queue_setup\n");
	
	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		sizes[0] = vdec_get_output_size(sess);
		sess->num_input_bufs = *num_buffers;
		*num_planes = fmt_out->num_planes;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		sizes[0] = vdec_get_output_size(sess);
		sizes[1] = vdec_get_output_size(sess) / 2;
		*num_buffers = min(max(*num_buffers, fmt_out->min_buffers), fmt_out->max_buffers);
		sess->num_output_bufs = *num_buffers;
		*num_planes = fmt_cap->num_planes;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void vdec_queue_recycle(struct vdec_session *sess, struct vb2_buffer *vb)
{
	struct vdec_buffer *new_buf;

	new_buf = kmalloc(sizeof(struct vdec_buffer), GFP_KERNEL);
	new_buf->index = vb->index;

	mutex_lock(&sess->bufs_recycle_lock);
	list_add_tail(&new_buf->list, &sess->bufs_recycle);
	mutex_unlock(&sess->bufs_recycle_lock);
}

static void vdec_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vdec_session *sess = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = sess->m2m_ctx;

	mutex_lock(&sess->lock);
	v4l2_m2m_buf_queue(m2m_ctx, vbuf);

	if (!(sess->streamon_out & sess->streamon_cap))
		goto unlock;
	
	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		sess->input_bufs_ready = 1;
		wake_up_interruptible(&sess->input_buf_wq);
	}
	else
		vdec_queue_recycle(sess, vb);

unlock:
	mutex_unlock(&sess->lock);
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vdec_session *sess = vb2_get_drv_priv(q);
	int ret;
	
	printk("vdec_start_streaming\n");
	mutex_lock(&sess->lock);
	
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->streamon_out = 1;
	else
		sess->streamon_cap = 1;

	if (!(sess->streamon_out & sess->streamon_cap)) {
		mutex_unlock(&sess->lock);
		return 0;
	}
	
	/* Allocate 32 MiB for the VIFIFO buffer */
	sess->vififo_size = 0x2000000;
	sess->vififo_vaddr = dma_alloc_coherent(sess->core->dev, sess->vififo_size, &sess->vififo_paddr, GFP_KERNEL);
	if (!sess->vififo_vaddr) {
		printk("Failed to request 32MiB VIFIFO buffer\n");
		ret = -ENOMEM;
		goto bufs_done;
	}
	printk("Allocated 32MiB: %08X - %08X\n", sess->vififo_paddr, sess->vififo_paddr + sess->vififo_size);

	pm_runtime_get_sync(sess->core->dev_dec);
	ret = vdec_poweron(sess);
	if (ret)
		goto vififo_free;

	sess->sequence_cap = 0;

	printk("Launching thread\n");
	sess->esparser_queue_thread = kthread_run(esparser_queue, sess, "esparser_queue");
	printk("start_streaming done\n");
	mutex_unlock(&sess->lock);

	return 0;

vififo_free:
	dma_free_coherent(sess->core->dev, sess->vififo_size, sess->vififo_vaddr, sess->vififo_paddr);
bufs_done:
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->streamon_out = 0;
	else
		sess->streamon_cap = 0;
	mutex_unlock(&sess->lock);
	return ret;
}

void vdec_stop_streaming(struct vb2_queue *q)
{
	struct vdec_session *sess = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;

	printk("vdec_stop_streaming\n");
	mutex_lock(&sess->lock);

	if (sess->streamon_out & sess->streamon_cap) {
		vdec_poweroff(sess);
		pm_runtime_put_sync(sess->core->dev_dec);
		dma_free_coherent(sess->core->dev, sess->vififo_size, sess->vififo_vaddr, sess->vififo_paddr);
		INIT_LIST_HEAD(&sess->bufs);
		INIT_LIST_HEAD(&sess->bufs_recycle);
		sema_init(&sess->queue_sema, 24);
		sess->input_bufs_ready = 0;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((buf = v4l2_m2m_src_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		sess->streamon_out = 0;
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(sess->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		sess->streamon_cap = 0;
	}

	mutex_unlock(&sess->lock);
}

static const struct vb2_ops vdec_vb2_ops = {
	.queue_setup = vdec_queue_setup,
	/*.buf_init = vdec_vb2_buf_init,
	.buf_prepare = vdec_vb2_buf_prepare,*/
	.start_streaming = vdec_start_streaming,
	.stop_streaming = vdec_stop_streaming,
	.buf_queue = vdec_vb2_buf_queue,
};

static int
vdec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	printk("vdec_querycap\n");
	strlcpy(cap->driver, "meson-vdec", sizeof(cap->driver));
	strlcpy(cap->card, "AMLogic Video Decoder", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:meson-vdec", sizeof(cap->bus_info));

	return 0;
}

static const struct vdec_format vdec_formats[] = {
	{
		.pixfmt = V4L2_PIX_FMT_NV12M,
		.num_planes = 2,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 32,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = &codec_h264_ops,
		.firmware_path = "meson/gxl/gxtvbb_vh264_mc",
	}, {
		.pixfmt = V4L2_PIX_FMT_HEVC,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 32,
		.vdec_ops = &vdec_hevc_ops,
		.codec_ops = &codec_hevc_ops,
		.firmware_path = "meson/gxl/vh265_mc_mmu",
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG1,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 8,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = &codec_mpeg12_ops,
		.firmware_path = "meson/gxl/vmpeg12_mc",
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG2,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 8,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = &codec_mpeg12_ops,
		.firmware_path = "meson/gxl/vmpeg12_mc",
	}, {
		.pixfmt = V4L2_PIX_FMT_MPEG4,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 8,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = &codec_mpeg4_ops,
		.firmware_path = "meson/gxl/vmpeg4_mc_5",
	}, {
		.pixfmt = V4L2_PIX_FMT_H263,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 8,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = &codec_mpeg4_ops,
		.firmware_path = "meson/gxl/h263_mc",
	}, {
		.pixfmt = V4L2_PIX_FMT_XVID,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.min_buffers = 8,
		.max_buffers = 8,
		.vdec_ops = &vdec_1_ops,
		.codec_ops = &codec_mpeg4_ops,
		.firmware_path = "meson/gxl/vmpeg4_mc_5",
	},
};

static const struct vdec_format * find_format(u32 pixfmt, u32 type)
{
	const struct vdec_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	unsigned int i;

	for (i = 0; i < size; i++) {
		if (fmt[i].pixfmt == pixfmt)
			break;
	}

	if (i == size || fmt[i].type != type)
		return NULL;

	return &fmt[i];
}

static const struct vdec_format *
find_format_by_index(unsigned int index, u32 type)
{
	const struct vdec_format *fmt = vdec_formats;
	unsigned int size = ARRAY_SIZE(vdec_formats);
	unsigned int i, k = 0;

	if (index > size)
		return NULL;

	for (i = 0; i < size; i++) {
		if (fmt[i].type != type)
			continue;
		if (k == index)
			break;
		k++;
	}

	if (i == size)
		return NULL;

	return &fmt[i];
}

static const struct vdec_format *
vdec_try_fmt_common(struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = pixmp->plane_fmt;
	const struct vdec_format *fmt;

	memset(pfmt[0].reserved, 0, sizeof(pfmt[0].reserved));
	memset(pixmp->reserved, 0, sizeof(pixmp->reserved));

	fmt = find_format(pixmp->pixelformat, f->type);
	if (!fmt) {
		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_NV12M;
		else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
		else
			return NULL;
		fmt = find_format(pixmp->pixelformat, f->type);
		pixmp->width = 1280;
		pixmp->height = 720;
	}

	pixmp->width  = clamp(pixmp->width,  (u32)256, (u32)3840);
	pixmp->height = clamp(pixmp->height, (u32)144, (u32)2160);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		memset(pfmt[1].reserved, 0, sizeof(pfmt[1].reserved));
		pfmt[0].sizeimage = get_output_size(pixmp->width, pixmp->height);
		pfmt[0].bytesperline = ALIGN(pixmp->width, 64);

		pfmt[1].sizeimage = get_output_size(pixmp->width, pixmp->height) / 2;
		pfmt[1].bytesperline = ALIGN(pixmp->width, 64);
	} else {
		pfmt[0].sizeimage = get_output_size(pixmp->width, pixmp->height);
		pfmt[0].bytesperline = 0;
	}


	return fmt;
}

static int vdec_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	printk("vdec_try_fmt\n");
	vdec_try_fmt_common(f);

	return 0;
}

static int vdec_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vdec_session *sess = container_of(file->private_data, struct vdec_session, fh);
	const struct vdec_format *fmt = NULL;
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;

	printk("vdec_g_fmt\n");
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		fmt = sess->fmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = sess->fmt_out;

	pixmp->pixelformat = fmt->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
		pixmp->colorspace = sess->colorspace;
		pixmp->ycbcr_enc = sess->ycbcr_enc;
		pixmp->quantization = sess->quantization;
		pixmp->xfer_func = sess->xfer_func;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = sess->width;
		pixmp->height = sess->height;
	}

	vdec_try_fmt_common(f);

	return 0;
}

static int vdec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vdec_session *sess = container_of(file->private_data, struct vdec_session, fh);
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;
	struct v4l2_pix_format_mplane orig_pixmp;
	const struct vdec_format *fmt;
	struct v4l2_format format;
	u32 pixfmt_out = 0, pixfmt_cap = 0;

	printk("vdec_s_fmt\n");
	orig_pixmp = *pixmp;

	fmt = vdec_try_fmt_common(f);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixfmt_out = pixmp->pixelformat;
		pixfmt_cap = sess->fmt_cap->pixfmt;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = sess->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(&format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		sess->width = format.fmt.pix_mp.width;
		sess->height = format.fmt.pix_mp.height;
		sess->colorspace = pixmp->colorspace;
		sess->ycbcr_enc = pixmp->ycbcr_enc;
		sess->quantization = pixmp->quantization;
		sess->xfer_func = pixmp->xfer_func;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(&format);

	sess->width = format.fmt.pix_mp.width;
	sess->height = format.fmt.pix_mp.height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		sess->fmt_out = fmt;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		sess->fmt_cap = fmt;

	return 0;
}

static int vdec_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vdec_format *fmt;

	printk("vdec_enum_fmt\n");
	memset(f->reserved, 0, sizeof(f->reserved));

	fmt = find_format_by_index(f->index, f->type);
	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->pixfmt;

	return 0;
}

static int vdec_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	const struct vdec_format *fmt;

	fmt = find_format(fsize->pixel_format,
			  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	if (!fmt) {
		fmt = find_format(fsize->pixel_format,
				  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		if (!fmt)
			return -EINVAL;
	}

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;

	/* TODO: Store these constants in vdec_format */
	fsize->stepwise.min_width = 256;
	fsize->stepwise.max_width = 3840;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 144;
	fsize->stepwise.max_height = 2160;
	fsize->stepwise.step_height = 1;

	return 0;
}

static const struct v4l2_ioctl_ops vdec_ioctl_ops = {
	.vidioc_querycap = vdec_querycap,
	.vidioc_enum_fmt_vid_cap_mplane = vdec_enum_fmt,
	.vidioc_enum_fmt_vid_out_mplane = vdec_enum_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vdec_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vdec_s_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vdec_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vdec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vdec_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vdec_try_fmt,
	//.vidioc_g_selection = vdec_g_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	//.vidioc_s_parm = vdec_s_parm,
	.vidioc_enum_framesizes = vdec_enum_framesizes,
	//.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	//.vidioc_try_decoder_cmd = vdec_try_decoder_cmd,
	//.vidioc_decoder_cmd = vdec_decoder_cmd,
};

static int m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct vdec_session *sess = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vdec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = sess;
	src_vq->buf_struct_size = sizeof(struct dummy_buf);
	src_vq->allow_zero_bytesused = 1;
	src_vq->min_buffers_needed = 1;
	src_vq->dev = sess->core->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vdec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = sess;
	dst_vq->buf_struct_size = sizeof(struct dummy_buf);
	dst_vq->allow_zero_bytesused = 1;
	dst_vq->min_buffers_needed = 1;
	dst_vq->dev = sess->core->dev;
	ret = vb2_queue_init(dst_vq);
	if (ret) {
		vb2_queue_release(src_vq);
		return ret;
	}

	return 0;
}

static int vdec_open(struct file *file)
{
	struct vdec_core *core = video_drvdata(file);
	struct vdec_session *sess;
	
	sess = kzalloc(sizeof(*sess), GFP_KERNEL);
	if (!sess)
		return -ENOMEM;

	printk("vdec_open\n");

	sess->core = core;
	sess->fmt_cap = &vdec_formats[0];
	sess->fmt_out = &vdec_formats[1];
	sess->width = 1280;
	sess->height = 720;
	INIT_LIST_HEAD(&sess->bufs);
	INIT_LIST_HEAD(&sess->bufs_recycle);
	init_waitqueue_head(&sess->input_buf_wq);
	spin_lock_init(&sess->bufs_spinlock);
	mutex_init(&sess->lock);
	mutex_init(&sess->bufs_recycle_lock);
	sema_init(&sess->queue_sema, 24);

	core->cur_sess = sess;

	sess->m2m_dev = v4l2_m2m_init(&vdec_m2m_ops);
	if (IS_ERR(sess->m2m_dev)) {
		printk("Fail to v4l2_m2m_init\n");
		return PTR_ERR(sess->m2m_dev);
	}

	sess->m2m_ctx = v4l2_m2m_ctx_init(sess->m2m_dev, sess, m2m_queue_init);
	if (IS_ERR(sess->m2m_ctx)) {
		printk("Fail to v4l2_m2m_ctx_init\n");
		return PTR_ERR(sess->m2m_ctx);
	}

	v4l2_fh_init(&sess->fh, core->vdev_dec);
	v4l2_fh_add(&sess->fh);
	sess->fh.m2m_ctx = sess->m2m_ctx;
	file->private_data = &sess->fh;

	return 0;
}

static int vdec_close(struct file *file)
{
	struct vdec_session *sess = container_of(file->private_data, struct vdec_session, fh);

	printk("vdec_close\n");
	v4l2_m2m_ctx_release(sess->m2m_ctx);
	v4l2_m2m_release(sess->m2m_dev);
	v4l2_fh_del(&sess->fh);
	v4l2_fh_exit(&sess->fh);
	mutex_destroy(&sess->lock);

	kfree(sess);

	return 0;
}

static const struct v4l2_file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.open = vdec_open,
	.release = vdec_close,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = v4l2_compat_ioctl32,
#endif
};

static int vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct vdec_core *core;
	struct resource *r;
	int irq;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core) {
		printk("No memory for devm_kzalloc\n");
		return -ENOMEM;
	}

	core->dev = dev;
	platform_set_drvdata(pdev, core);

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dos");
	core->dos_base = devm_ioremap_resource(dev, r);
	if (IS_ERR(core->dos_base)) {
		printk("Couldn't remap DOS memory\n");
		return PTR_ERR(core->dos_base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "esparser");
	core->esparser_base = devm_ioremap_resource(dev, r);
	if (IS_ERR(core->esparser_base)) {
		printk("Couldn't remap ESPARSER memory\n");
		return PTR_ERR(core->esparser_base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dmc");
	core->dmc_base = devm_ioremap(dev, r->start, resource_size(r));
	if (IS_ERR(core->dmc_base)) {
		printk("Couldn't remap DMC memory\n");
		return PTR_ERR(core->dmc_base);
	}

	core->regmap_ao = syscon_regmap_lookup_by_phandle(dev->of_node, "amlogic,ao-sysctrl");
	if (IS_ERR(core->regmap_ao)) {
		printk("Couldn't regmap AO sysctrl\n");
		return PTR_ERR(core->regmap_ao);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
		
	ret = devm_request_irq(core->dev, irq, vdec_1_isr,
				IRQF_SHARED, "vdecirq", core);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret) {
		printk("Couldn't register v4l2 device\n");
		return -ENOMEM;
	}

	ret = esparser_init(pdev, core);

	vdev = video_device_alloc();
	if (!vdev)
		return -ENOMEM;

	strlcpy(vdev->name, "meson-video-decoder", sizeof(vdev->name));
	vdev->release = video_device_release;
	vdev->fops = &vdec_fops;
	vdev->ioctl_ops = &vdec_ioctl_ops;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->v4l2_dev = &core->v4l2_dev;
	vdev->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		printk("Failed registering video device\n");
		goto err_vdev_release;
	}

	core->vdev_dec = vdev;
	core->dev_dec = dev;

	video_set_drvdata(vdev, core);
	pm_runtime_enable(dev);

	return 0;

err_vdev_release:
	video_device_release(vdev);
	return ret;
}

static int vdec_remove(struct platform_device *pdev)
{
	struct vdec_core *core = dev_get_drvdata(pdev->dev.parent);

	video_unregister_device(core->vdev_dec);

	return 0;
}

static const struct of_device_id vdec_dt_match[] = {
	{ .compatible = "amlogic,meson8b-vdec" },
	{ }
};
MODULE_DEVICE_TABLE(of, vdec_dt_match);

static struct platform_driver meson_vdec_driver = {
	.probe = vdec_probe,
	.remove = vdec_remove,
	.driver = {
		.name = "meson-vdec",
		.of_match_table = vdec_dt_match,
	},
};
module_platform_driver(meson_vdec_driver);

MODULE_ALIAS("platform:meson-video-decoder");
MODULE_DESCRIPTION("AMLogic Meson8(b) video decoder driver");
MODULE_AUTHOR("Maxime Jourdan <maxi.jourdan@wanadoo.fr>");
MODULE_LICENSE("GPL v2");
