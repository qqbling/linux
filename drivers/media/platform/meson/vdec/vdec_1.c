#include "vdec_1.h"

/* AO Registers */
#define AO_RTI_GEN_PWR_ISO0 0xec

/* DOS Registers */
#define ASSIST_MBOX1_CLR_REG 0x01d4
#define ASSIST_MBOX1_MASK    0x01d8

#define IMEM_DMA_CTRL  0x0d00
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
#define DOS_MEM_PD_VDEC           0xfcc0
#define DOS_VDEC_MCRCC_STALL_CTRL 0xfd00

irqreturn_t vdec_1_isr(int irq, void *data)
{
	struct vdec_core *core = data;
	struct vdec_session *sess = core->cur_sess;
	return sess->fmt_out->codec_ops->isr(sess);
}

static int vdec_1_start(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	int ret;

	printk("vdec_1_start\n");
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

	return 0;
}

static int vdec_1_stop(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	printk("vdec_1_stop\n");

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

	printk("vdec_poweroff end\n");
	return 0;
}

struct vdec_ops vdec_1_ops = {
	.start = vdec_1_start,
	.stop = vdec_1_stop,
};