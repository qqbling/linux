#include <linux/clk.h>
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
#include <linux/firmware.h>

#include "vdec.h"
#include "esparser.h"
#include "canvas.h"

#define MC_SIZE			(4096 * 4)
#define MC_H264_EXT_SIZE	(4096 * 5)
#define MAX_DPB_BUFF_SIZE	(12*1024*1024) // Big enough for a 3840*2160 4:2:0 buffer
#define DEF_BUF_START_ADDR	0x1000000
#define V_BUF_ADDR_OFFSET	0x13e000

#define UNK1_SIZE 0x13e000
#define REF_SIZE  0x100000
#define UNK2_SIZE 0x300000

/* DOS registers */
#define ASSIST_MBOX1_CLR_REG 0x01d4
#define ASSIST_MBOX1_MASK    0x01d8

#define MPSR 0x0c04
#define CPSR 0x0c84

#define MC_STATUS0  0x2424
#define MC_CTRL1    0x242c
#define PSCALE_CTRL 0x2444

#define DBLK_CTRL   0x2544
#define DBLK_STATUS 0x254c

#define MDEC_PIC_DC_CTRL   0x2638
#define MDEC_PIC_DC_STATUS 0x263c
#define ANC0_CANVAS_ADDR   0x2640
#define MDEC_PIC_DC_THRESH 0x26e0

#define AV_SCRATCH_0  0x2700
#define AV_SCRATCH_1  0x2704
#define AV_SCRATCH_2  0x2708
#define AV_SCRATCH_3  0x270c
#define AV_SCRATCH_4  0x2710
#define AV_SCRATCH_5  0x2714

#define AV_SCRATCH_6  0x2718
#define AV_SCRATCH_7  0x271c
#define AV_SCRATCH_8  0x2720
#define AV_SCRATCH_9  0x2724
#define AV_SCRATCH_D  0x2734
#define AV_SCRATCH_F  0x273c
#define AV_SCRATCH_G  0x2740

#define POWER_CTL_VLD 0x3020

#define DOS_SW_RESET0 0xfc00

static int vh264_load_extended_firmware(struct vdec_core *core, const struct firmware *fw) {
	core->vh264_ext_fw_vaddr = dma_alloc_coherent(NULL, MC_H264_EXT_SIZE, &core->vh264_ext_fw_paddr, GFP_KERNEL);

	if (!core->vh264_ext_fw_vaddr) {
		printk("Couldn't allocate memory for H.264 extended firmware\n");
		return -ENOMEM;
	}

	memcpy(core->vh264_ext_fw_vaddr, fw->data + MC_SIZE, MC_H264_EXT_SIZE);

	return 0;
}

/**
 * Load a VDEC firmware, each codec having its own firmware.
 * Some codecs also require additional firmware parts to be loaded after this
 */
static int vdec_load_firmware(struct vdec_core *core, const char* fwname)
{
	const struct firmware *fw;
	struct device *dev = core->dev_dec;
	static void *mc_addr;
	static dma_addr_t mc_addr_map;
	int ret;
	u32 i = 10000;

	ret = request_firmware(&fw, fwname, dev);
	if (ret < 0)  {
		dev_err(dev, "Unable to request firmware %s\n", fwname);
		return -EINVAL;
	}

	mc_addr = kmalloc(MC_SIZE, GFP_KERNEL);
	if (!mc_addr)
		return -ENOMEM;

	memcpy(mc_addr, fw->data, MC_SIZE);
	mc_addr_map = dma_map_single(NULL, mc_addr, MC_SIZE, DMA_TO_DEVICE);
	if (!mc_addr_map) {
		dev_err(dev, "Couldn't MAP DMA addr\n");
		return -EINVAL;
	}

	writel_relaxed(1, core->dos_base + MPSR);
	writel_relaxed(1, core->dos_base + CPSR);

	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) & ~(1<<31), core->dos_base + MDEC_PIC_DC_CTRL);
	writel_relaxed(mc_addr_map, core->dos_base + 0xd04); // IMEM_DMA_ADR
	writel_relaxed(MC_SIZE, core->dos_base + 0xd08); // IMEM_DMA_COUNT
	writel_relaxed((0x8000 | (7 << 16)), core->dos_base + 0xd00); // IMEM_DMA_CTRL ; Magic value from AML code

	printk("Starting wait..\n");
	while (--i && readl(core->dos_base + 0xd00) & 0x8000) { }

	if (i == 0) {
		printk("Firmware load fail (DMA hang?)\n");
		ret = -EINVAL;
	} else
		printk("Firmware load success\n");

	vh264_load_extended_firmware(core, fw);

	dma_unmap_single(NULL, mc_addr_map, MC_SIZE, DMA_TO_DEVICE);
	kfree(mc_addr);
	release_firmware(fw);
	return ret;
}

static void vdec_abort(struct vdec_core *core) {
	printk("Aborting decoding session!\n");
	vb2_queue_error(&core->m2m_ctx->cap_q_ctx.q);
	vb2_queue_error(&core->m2m_ctx->out_q_ctx.q);
}

static void vh264_power_up(struct vdec_core *core) {
	/* Taken from old AMLogic code. No idea. */
	writel_relaxed((1<<7) | (1<<6) | (1<<4), core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);

	writel_relaxed((1<<7) | (1<<6) | (1<<4), core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET0);
	writel_relaxed((1<<9) | (1<<8), core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);

	writel_relaxed(readl_relaxed(core->dos_base + POWER_CTL_VLD) | (1 << 9) | (1 << 6), core->dos_base + POWER_CTL_VLD);

	writel_relaxed(0, core->dos_base + PSCALE_CTRL);

	writel_relaxed(0, core->dos_base + AV_SCRATCH_0);
	writel_relaxed(core->vh264_mem_paddr - DEF_BUF_START_ADDR, core->dos_base + AV_SCRATCH_1); // buf offset (?)
	writel_relaxed(core->vh264_ext_fw_paddr, core->dos_base + AV_SCRATCH_G); // ext. firmware addr
	writel_relaxed(0, core->dos_base + AV_SCRATCH_7);
	writel_relaxed(0, core->dos_base + AV_SCRATCH_8);
	writel_relaxed(0, core->dos_base + AV_SCRATCH_9);

	/* Enable "error correction", don't know what it means */
	writel_relaxed((readl_relaxed(core->dos_base + AV_SCRATCH_F) & 0xffffffc3) | (1 << 4), core->dos_base + AV_SCRATCH_F);

	/* Enable IRQ */
	writel_relaxed(1, core->dos_base + ASSIST_MBOX1_CLR_REG);
	writel_relaxed(1, core->dos_base + ASSIST_MBOX1_MASK);

	/* Enable NV21 */
	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) | (1 << 17), core->dos_base + MDEC_PIC_DC_CTRL);

	//writel_relaxed(readl_relaxed(core->dos_base + AV_SCRATCH_F) | (1 << 6), core->dos_base + AV_SCRATCH_F);

	writel_relaxed(0x404038aa, core->dos_base + MDEC_PIC_DC_THRESH);
}

static u32 get_output_size(u32 width, u32 height) {
	return ALIGN(width, 64) * ALIGN(height, 64);
}

static u32 vdec_get_output_size(struct vdec_core *core) {
	return get_output_size(core->width, core->height);
}

static int vdec_poweron(struct vdec_core *core) {
	int ret;

	printk("vdec_poweron\n");

	/* Reset VDEC1 */
	writel(0xfffffffc, core->dos_base + DOS_SW_RESET0);
	writel(0x00000000, core->dos_base + DOS_SW_RESET0);

	writel(0x3ff, core->dos_base + 0xfc04); // DOS_GCLK_EN0

	/* VDEC Memories */
	writel(0x00000000, core->dos_base + 0xfcc0);

	/* Remove VDEC1 Isolation */
	regmap_write(core->regmap_ao, 0xec, 0x00000000);

	/* Reset DOS top registers */
	writel(0x00000000, core->dos_base + 0xfd00);

	writel_relaxed(0x3ff, core->dos_base + 0x260c); // GCLK_EN

	stbuf_power_up(core);

	/*TODO: power up the decoder related to the input PIXFMT */
	ret = vdec_load_firmware(core, "meson/m8/vh264_mc");
	if (ret)
		return ret;

	vh264_power_up(core);

	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);

	writel_relaxed((1<<12)|(1<<11), core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET0);

	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);

	/* Enable VDEC? */
	writel_relaxed(1, core->dos_base + MPSR);

	esparser_power_up(core);

	return 0;
}

static void vdec_poweroff(struct vdec_core *core) {
	printk("vdec_poweroff\n");

	writel_relaxed(0, core->dos_base + MPSR);
	writel_relaxed(0, core->dos_base + CPSR);

	writel_relaxed((1<<12)|(1<<11), core->dos_base + DOS_SW_RESET0);
	writel_relaxed(0, core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);
	readl_relaxed(core->dos_base + DOS_SW_RESET0);

	writel_relaxed(0, core->dos_base + ASSIST_MBOX1_MASK);

	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) | 1, core->dos_base + MDEC_PIC_DC_CTRL);
	writel_relaxed(readl_relaxed(core->dos_base + MDEC_PIC_DC_CTRL) & ~1, core->dos_base + MDEC_PIC_DC_CTRL);
	readl_relaxed(core->dos_base + MDEC_PIC_DC_STATUS);
	readl_relaxed(core->dos_base + MDEC_PIC_DC_STATUS);

	writel_relaxed(3, core->dos_base + DBLK_CTRL);
	writel_relaxed(0, core->dos_base + DBLK_CTRL);
	readl_relaxed(core->dos_base + DBLK_STATUS);
	readl_relaxed(core->dos_base + DBLK_STATUS);

	writel_relaxed(readl_relaxed(core->dos_base + MC_CTRL1) | 0x9, core->dos_base + MC_CTRL1);
	writel_relaxed(readl_relaxed(core->dos_base + MC_CTRL1) & ~0x9, core->dos_base + MC_CTRL1);
	readl_relaxed(core->dos_base + MC_STATUS0);
	readl_relaxed(core->dos_base + MC_STATUS0);

	/* enable vdec1 isolation */
	regmap_write(core->regmap_ao, 0xec, 0xc0);
	/* power off vdec1 memories */
	writel(0xffffffffUL, core->dos_base + 0xfcc0);

	printk("vdec_poweroff end\n");
}

void vdec_m2m_device_run(void *priv) {
	struct vdec_core *core = priv;
	struct v4l2_m2m_buffer *buf, *n;

	printk("vdec_m2m_device_run\n");
	mutex_lock(&core->lock);

	v4l2_m2m_for_each_src_buf_safe(core->m2m_ctx, buf, n) {
		esparser_process_buf(core, &buf->vb);
	}

	mutex_unlock(&core->lock);
}

void vdec_m2m_job_abort(void *priv) {
	struct vdec_core *core = priv;

	printk("vdec_m2m_job_abort\n");
	v4l2_m2m_job_finish(core->m2m_dev, core->m2m_ctx);
}

static const struct v4l2_m2m_ops vdec_m2m_ops = {
	.device_run = vdec_m2m_device_run,
	.job_abort = vdec_m2m_job_abort,
};

static int vdec_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct vdec_core *core = vb2_get_drv_priv(q);
	printk("vdec_queue_setup\n");
	
	switch (q->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		sizes[0] = vdec_get_output_size(core);
		*num_buffers = 1;
		*num_planes = 1;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		sizes[0] = vdec_get_output_size(core);
		sizes[1] = vdec_get_output_size(core) / 2;
		*num_buffers = 24;
		*num_planes = 2;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void vdec_queue_recycle(struct vdec_core *core, struct vb2_buffer *vb)
{
	struct vdec_buffer *new_buf;

	new_buf = kmalloc(sizeof(struct vdec_buffer), GFP_KERNEL);
	new_buf->index = vb->index;

	list_add_tail(&new_buf->list, &core->bufs_recycle);
}

static void vdec_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vdec_core *core = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_m2m_ctx *m2m_ctx = core->m2m_ctx;

	mutex_lock(&core->lock);
	v4l2_m2m_buf_queue(m2m_ctx, vbuf);

	if (!(core->streamon_out & core->streamon_cap))
		goto unlock;
	
	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		esparser_process_buf(core, vbuf);
	else
		vdec_queue_recycle(core, vb);

unlock:
	mutex_unlock(&core->lock);
}

static int mark_buffers_done(void *data)
{
	struct vdec_core *core = data;
	struct vdec_buffer *tmp;
	struct vb2_v4l2_buffer *vbuf;
	unsigned long flags;

	while (!kthread_should_stop()) {
		while (!list_empty(&core->bufs))
		{
			tmp = list_first_entry(&core->bufs, struct vdec_buffer, list);
			if (tmp->index == -1)
				break;

			vbuf = v4l2_m2m_dst_buf_remove_by_idx(core->m2m_ctx, tmp->index);
			if (!vbuf) {
				printk("HW buffer ready but we don't have the vb2 buffer !!!\n");
				continue;
			}

			vbuf->vb2_buf.timestamp = tmp->timestamp;
			vbuf->sequence = core->sequence_cap++;
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);

			//printk("Recycled buf %d ; seq = %d ; flags = %08X ; (timestamp %llu)\n", tmp->index, vbuf->sequence, vbuf->flags, vbuf->vb2_buf.timestamp);

			spin_lock_irqsave(&core->bufs_spinlock, flags);
			list_del(&tmp->list);
			spin_unlock_irqrestore(&core->bufs_spinlock, flags);
			kfree(tmp);
		}

		while (!list_empty(&core->bufs_recycle) &&
		      (!readl_relaxed(core->dos_base + AV_SCRATCH_7) ||
		       !readl_relaxed(core->dos_base + AV_SCRATCH_8)))
		{
			tmp = list_first_entry(&core->bufs_recycle, struct vdec_buffer, list);

			/* Tell the decoder he can recycle this buffer.
			 * AV_SCRATCH_8 serves the same purpose.
			 */
			if (!readl_relaxed(core->dos_base + AV_SCRATCH_7))
				writel_relaxed(tmp->index + 1, core->dos_base + AV_SCRATCH_7);
			else
				writel_relaxed(tmp->index + 1, core->dos_base + AV_SCRATCH_8);

			list_del(&tmp->list);
			kfree(tmp);

			up(&core->queue_sema);
		}

		msleep(20);
	}

	return 0;
}

static int vdec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vdec_core *core = vb2_get_drv_priv(q);
	int ret;
	
	printk("vdec_start_streaming\n");
	mutex_lock(&core->lock);
	
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		core->streamon_out = 1;
	else
		core->streamon_cap = 1;

	if (!(core->streamon_out & core->streamon_cap)) {
		mutex_unlock(&core->lock);
		return 0;
	}

	ret = vdec_poweron(core);
	if (ret)
		goto bufs_done;

	core->sequence_cap = 0;

	printk("Launching thread\n");
	core->buffers_done_thread = kthread_run(mark_buffers_done, core, "buffers_done");
	printk("start_streaming done\n");
	mutex_unlock(&core->lock);

	return 0;

bufs_done:
	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		core->streamon_out = 0;
	else
		core->streamon_cap = 0;
	mutex_unlock(&core->lock);
	return ret;
}

void vdec_stop_streaming(struct vb2_queue *q)
{
	struct vdec_core *core = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;

	mutex_lock(&core->lock);

	if (core->streamon_out & core->streamon_cap) {
		kthread_stop(core->buffers_done_thread);
		vdec_poweroff(core);
		INIT_LIST_HEAD(&core->bufs);
		INIT_LIST_HEAD(&core->bufs_recycle);
		sema_init(&core->queue_sema, 20);
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((buf = v4l2_m2m_src_buf_remove(core->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		core->streamon_out = 0;
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(core->m2m_ctx)))
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);

		core->streamon_cap = 0;
	}

	mutex_unlock(&core->lock);
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
		.pixfmt = V4L2_PIX_FMT_NV21M,
		.num_planes = 2,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	}, {
		.pixfmt = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
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
			pixmp->pixelformat = V4L2_PIX_FMT_NV21M;
		else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			pixmp->pixelformat = V4L2_PIX_FMT_H264;
		else
			return NULL;
		fmt = find_format(pixmp->pixelformat, f->type);
		pixmp->width = 1280;
		pixmp->height = 720;
	}

	pixmp->width  = clamp(pixmp->width,  (u32)256, (u32)1920);
	pixmp->height = clamp(pixmp->height, (u32)144, (u32)1080);

	if (pixmp->field == V4L2_FIELD_ANY)
		pixmp->field = V4L2_FIELD_NONE;

	pixmp->num_planes = fmt->num_planes;
	pixmp->flags = 0;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
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
	struct vdec_core *core = container_of(file->private_data, struct vdec_core, fh);
	const struct vdec_format *fmt = NULL;
	struct v4l2_pix_format_mplane *pixmp = &f->fmt.pix_mp;

	printk("vdec_g_fmt\n");
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		fmt = core->fmt_cap;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		fmt = core->fmt_out;

	pixmp->pixelformat = fmt->pixfmt;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixmp->width = core->width;
		pixmp->height = core->height;
		pixmp->colorspace = core->colorspace;
		pixmp->ycbcr_enc = core->ycbcr_enc;
		pixmp->quantization = core->quantization;
		pixmp->xfer_func = core->xfer_func;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		pixmp->width = core->width;
		pixmp->height = core->height;
	}

	vdec_try_fmt_common(f);

	return 0;
}

static int vdec_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vdec_core *core = container_of(file->private_data, struct vdec_core, fh);
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
		pixfmt_cap = core->fmt_cap->pixfmt;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pixfmt_cap = pixmp->pixelformat;
		pixfmt_out = core->fmt_out->pixfmt;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_out;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(&format);

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		core->width = format.fmt.pix_mp.width;
		core->height = format.fmt.pix_mp.height;
		core->colorspace = pixmp->colorspace;
		core->ycbcr_enc = pixmp->ycbcr_enc;
		core->quantization = pixmp->quantization;
		core->xfer_func = pixmp->xfer_func;
	}

	memset(&format, 0, sizeof(format));

	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format.fmt.pix_mp.pixelformat = pixfmt_cap;
	format.fmt.pix_mp.width = orig_pixmp.width;
	format.fmt.pix_mp.height = orig_pixmp.height;
	vdec_try_fmt_common(&format);

	core->width = format.fmt.pix_mp.width;
	core->height = format.fmt.pix_mp.height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		core->fmt_out = fmt;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		core->fmt_cap = fmt;

	return 0;
}

static int vdec_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	printk("vdec_enum_fmt\n");
	memset(f->reserved, 0, sizeof(f->reserved));

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		f->pixelformat = V4L2_PIX_FMT_H264;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		f->pixelformat = V4L2_PIX_FMT_NV21M;
	else
		return -EINVAL;

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
	//.vidioc_enum_framesizes = vdec_enum_framesizes,
	//.vidioc_subscribe_event = vdec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	//.vidioc_try_decoder_cmd = vdec_try_decoder_cmd,
	//.vidioc_decoder_cmd = vdec_decoder_cmd,
};

static int m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			  struct vb2_queue *dst_vq)
{
	struct vdec_core *core = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &vdec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = core;
	src_vq->buf_struct_size = sizeof(struct dummy_buf);
	src_vq->allow_zero_bytesused = 1;
	src_vq->min_buffers_needed = 1;
	src_vq->dev = core->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &vdec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = core;
	dst_vq->buf_struct_size = sizeof(struct dummy_buf);
	dst_vq->allow_zero_bytesused = 1;
	dst_vq->min_buffers_needed = 1;
	dst_vq->dev = core->dev;
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

	printk("vdec_open\n");
	pm_runtime_get_sync(core->dev_dec);

	core->m2m_dev = v4l2_m2m_init(&vdec_m2m_ops);
	if (IS_ERR(core->m2m_dev)) {
		printk("Fail to v4l2_m2m_init\n");
		return PTR_ERR(core->m2m_dev);
	}

	core->m2m_ctx = v4l2_m2m_ctx_init(core->m2m_dev, core, m2m_queue_init);
	if (IS_ERR(core->m2m_ctx)) {
		printk("Fail to v4l2_m2m_ctx_init\n");
		return PTR_ERR(core->m2m_ctx);
	}

	v4l2_fh_init(&core->fh, core->vdev_dec);
	//core->fh.ctrl_handler = &core->ctrl_handler;
	v4l2_fh_add(&core->fh);
	core->fh.m2m_ctx = core->m2m_ctx;
	file->private_data = &core->fh;

	return 0;
}

static int vdec_close(struct file *file)
{
	struct vdec_core *core = container_of(file->private_data, struct vdec_core, fh);

	printk("vdec_close\n");
	v4l2_m2m_ctx_release(core->m2m_ctx);
	v4l2_m2m_release(core->m2m_dev);
	v4l2_fh_del(&core->fh);
	v4l2_fh_exit(&core->fh);

	pm_runtime_put_sync(core->dev_dec);

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

/* Configure the H.264 decoder when the esparser finished parsing
 * the first buffer.
 * TODO: move this to a specific H.264 subdevice file
 */
static void vdec_set_param(struct vdec_core *core) {
	u32 max_reference_size;
	u32 parsed_info, mb_width, mb_height, mb_total;
	u32 mb_mv_byte;
	u32 addr;
	u32 actual_dpb_size = v4l2_m2m_num_dst_bufs_ready(core->m2m_ctx);
	struct v4l2_m2m_buffer *buf;

	writel_relaxed(0, core->dos_base + AV_SCRATCH_7);
	writel_relaxed(0, core->dos_base + AV_SCRATCH_8);
	writel_relaxed(0, core->dos_base + AV_SCRATCH_9);

	printk("vdec_set_param\n");

	parsed_info = readl_relaxed(core->dos_base + AV_SCRATCH_1);

	/* Total number of 16x16 macroblocks */
	mb_total = (parsed_info >> 8) & 0xffff;

	/* Size of Motion Vector per macroblock ? */
	mb_mv_byte = (parsed_info & 0x80000000) ? 24 : 96;

	/* Number of macroblocks per line */
	mb_width = parsed_info & 0xff;

	/* Number of macroblock lines */
	mb_height = mb_total / mb_width;

	max_reference_size = (parsed_info >> 24) & 0x7f;

	/* Align to a multiple of 4 macroblocks */
	mb_width = (mb_width + 3) & 0xfffffffc;
	mb_height = (mb_height + 3) & 0xfffffffc;
	mb_total = mb_width * mb_height;

	/* Setup NV21 canvases for Decoded Picture Buffer (dpb)
	 * Map them to the user buffers' planes
	 */
	printk("Configuring %d canvases..\n", actual_dpb_size*2);
	v4l2_m2m_for_each_dst_buf(core->m2m_ctx, buf) {
		u32 buf_idx    = buf->vb.vb2_buf.index;
		u32 cnv_y_idx  = 128 + buf_idx * 2;
		u32 cnv_uv_idx = 128 + buf_idx * 2 + 1;
		dma_addr_t buf_y_paddr  = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		dma_addr_t buf_uv_paddr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 1);

		/* Y plane */
		vdec_canvas_setup(core->dmc_base, cnv_y_idx, buf_y_paddr, mb_width * 16, mb_height * 16, MESON_CANVAS_WRAP_NONE, MESON_CANVAS_BLKMODE_LINEAR);

		/* U/V plane */
		vdec_canvas_setup(core->dmc_base, cnv_uv_idx, buf_uv_paddr, mb_width * 16, mb_height * 8, MESON_CANVAS_WRAP_NONE, MESON_CANVAS_BLKMODE_LINEAR);

		writel_relaxed(((cnv_uv_idx) << 16) |
			       ((cnv_uv_idx) << 8)  |
				(cnv_y_idx), core->dos_base + ANC0_CANVAS_ADDR + buf_idx*4);
	}

	/* I don't really know the purpose of this post canvas.
	 * It seems required with the write to AV_SCRATCH_3 though..
	 */
	printk("Configuring post canvas to %08X\n", core->dummy_post_canvas_paddr);
	/* Setup post canvas for Y */
	vdec_canvas_setup(core->dmc_base, 0x00, core->dummy_post_canvas_paddr, mb_width << 4, mb_height << 4, MESON_CANVAS_WRAP_NONE, MESON_CANVAS_BLKMODE_LINEAR);
	/* Setup post canvas for U/V */
	vdec_canvas_setup(core->dmc_base, 0x1, core->dummy_post_canvas_paddr + (mb_total << 8), mb_width << 4, mb_height << 3, MESON_CANVAS_WRAP_NONE, MESON_CANVAS_BLKMODE_LINEAR);

	printk("mb_total = %d; mb_mv_byte = %d; actual_dpb_size = %d;\n max_reference_size = %d; mb_width = %d; mb_height = %d\n", mb_total, mb_mv_byte, actual_dpb_size, max_reference_size, mb_width, mb_height);

	printk("Setting POST CANVAS to %08X\n", (0x1 << 16) | (0x1 << 8) | 0x0);
	writel_relaxed((0x1 << 16) | (0x1 << 8) | 0x0, core->dos_base + AV_SCRATCH_3);

	/* Address to store the references' MVs ? */
	addr = core->vh264_mem_paddr + V_BUF_ADDR_OFFSET;
	writel_relaxed(addr, core->dos_base + AV_SCRATCH_1);
	printk("Max references buffer size: %d\n", mb_total * mb_mv_byte * max_reference_size);

	/* End of ref MV or start of something else ? */
	addr += mb_total * mb_mv_byte * max_reference_size;
	writel_relaxed(addr, core->dos_base + AV_SCRATCH_4);
	printk("Remaining buffer size: %d\n", core->vh264_mem_paddr + core->vh264_mem_size - addr);

	/* Hardcode max_dpb_size to 4 because I'm not sure what it is */
	writel_relaxed((max_reference_size << 24) | (actual_dpb_size << 16) | (4 << 8), core->dos_base + AV_SCRATCH_0);
}

/* Map a ready HW buffer index with a previously queued OUTPUT buffer's timestamp */
static void fill_buffer_index(struct vdec_core *core, u32 buffer_index) {
	struct vdec_buffer *tmp;
	unsigned long flags;

	spin_lock_irqsave(&core->bufs_spinlock, flags);
	list_for_each_entry(tmp, &core->bufs, list) {
		if (tmp->index == -1) {
			tmp->index = buffer_index;
			break;
		}
	}
	spin_unlock_irqrestore(&core->bufs_spinlock, flags);
}

static irqreturn_t vdec_isr(int irq, void *dev)
{
	unsigned int cpu_cmd;
	unsigned int buffer_index;
	int i;
	struct vdec_core *core = dev;

	writel_relaxed(1, core->dos_base + ASSIST_MBOX1_CLR_REG);
	cpu_cmd = readl_relaxed(core->dos_base + AV_SCRATCH_0);

	//printk("vdec_isr ; cpu_cmd = %08X!\n", cpu_cmd);

	if ((cpu_cmd & 0xff) == 1) {
		printk("calling vdec_set_param\n");
		vdec_set_param(core);
	} else if ((cpu_cmd & 0xff) == 2) {
		int error_count, error, num_frame, status, eos = 0;
		error_count = readl_relaxed(core->dos_base + AV_SCRATCH_D);
		num_frame = (cpu_cmd >> 8) & 0xff;
		if (error_count) {
			printk("decoder error(s) happened, count %d\n", error_count);
		}

		//printk("Decoded %d frames\n", num_frame);

		for (i = 0 ; (i < num_frame) && (!eos) ; i++) {

			status = readl_relaxed(core->dos_base + AV_SCRATCH_1 + i*4);
			buffer_index = status & 0x1f;
			error = status & 0x200;

			if (error) {
				printk("Buffer %d decode error: %08X\n", buffer_index, error);
			} else {
				//printk("Buffer %d decoded & ready!\n", buffer_index);
			}

			eos = (status >> 15) & 1;
		
			if (eos) {
				printk("Reached EOS!\n");
			}

			/* Fatal error ? */
			if (buffer_index >= 24) {
				printk("buffer_index >= 24 !! (%u)\n", buffer_index);
				continue;
			}

			fill_buffer_index(core, buffer_index);
		}

		writel_relaxed(0, core->dos_base + AV_SCRATCH_0);
		//schedule_work(&core->mark_buffers_done_work);
	} else {
		printk("Unexpected cpu_cmd: %08X\n", cpu_cmd);
		writel_relaxed(0, core->dos_base + AV_SCRATCH_0);
	}
	
	return IRQ_HANDLED;
}

static int vdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct video_device *vdev;
	struct vdec_core *core;
	struct resource *r;
	int ret;
	int irq;

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
	core->dmc_base = devm_ioremap_resource(dev, r);
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

	ret = devm_request_irq(dev, irq, vdec_isr,
					IRQF_SHARED,
					"vdecirq", core);
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret) {
		printk("Couldn't register v4l2 device\n");
		return -ENOMEM;
	}

	/* TODO: do the allocations at start_stream to not hog memory */

	/* Allocate 64 MiB for the DPB buffers <-- Obsolete since we map the canvases to the user buffers
	 * TODO: Pretty sure most of that chunk can be directly mapped to user buffers? must test.
	*/
	/*core->dpb_size = 0x4000000;
	core->dpb_vaddr = dma_alloc_coherent(NULL, core->dpb_size, &core->dpb_paddr, GFP_KERNEL);
	if (!core->dpb_vaddr) {
		printk("Failed to request 64MiB DPB video buffer\n");
		return -ENOMEM;
	}
	printk("Allocated 64MiB: %08X - %08X\n", core->dpb_paddr, core->dpb_paddr + core->dpb_size);*/

	/* Allocate some memory for the H.264 decoder's state
	 * (references motion vectors, and other things)
	 * TODO: move this to a specific H.264 subdevice file
	 */
	core->vh264_mem_size = UNK1_SIZE + REF_SIZE + UNK2_SIZE;
	core->vh264_mem_vaddr = dma_alloc_coherent(NULL, core->vh264_mem_size, &core->vh264_mem_paddr, GFP_KERNEL);
	if (!core->vh264_mem_vaddr) {
		printk("Failed to request 5.24MiB H.264 extra memory\n");
		return -ENOMEM;
	}
	printk("Allocated 5.24MiB: %08X - %08X\n", core->vh264_mem_paddr, core->vh264_mem_paddr + core->vh264_mem_size);

	/* Allocate 32 MiB for the VIFIFO buffer */
	core->vififo_size = 0x2000000;
	core->vififo_vaddr = dma_alloc_coherent(NULL, core->vififo_size, &core->vififo_paddr, GFP_KERNEL);
	if (!core->vififo_vaddr) {
		printk("Failed to request 32MiB VIFOFO buffer\n");
		return -ENOMEM;
	}
	printk("Allocated 32MiB: %08X - %08X\n", core->vififo_paddr, core->vififo_paddr + core->vififo_size);

	ret = esparser_init(pdev, core);

	/* Allocate a "post canvas", purpose unknown
	 * TODO: move this to a specific H.264 subdevice file
	 */
	core->dummy_post_canvas_vaddr = dma_alloc_coherent(NULL, 0x400000, &core->dummy_post_canvas_paddr, GFP_KERNEL);
	if (!core->dummy_post_canvas_paddr) {
		printk("Failed to request 4MiB post canvas\n");
		return -ENOMEM;
	}
	printk("Allocated 4MiB: %08X - %08X\n", core->dummy_post_canvas_paddr, core->dummy_post_canvas_paddr + 0x400000);

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

	INIT_LIST_HEAD(&core->bufs);
	INIT_LIST_HEAD(&core->bufs_recycle);
	//INIT_WORK(&core->mark_buffers_done_work, mark_buffers_done);
	spin_lock_init(&core->bufs_spinlock);
	mutex_init(&core->lock);
	sema_init(&core->queue_sema, 20);

	core->fmt_cap = &vdec_formats[0];
	core->fmt_out = &vdec_formats[1];

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
