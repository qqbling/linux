#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "codec_hevc.h"
#include "canvas.h"
#include "hevc_regs.h"

/* DOS registers */
#define ASSIST_MBOX1_CLR_REG 0x01d4
#define ASSIST_MBOX1_MASK    0x01d8

#define DOS_SW_RESET3        0xfcd0

/* HEVC reg mapping */
#define HEVC_DEC_STATUS_REG       HEVC_ASSIST_SCRATCH_0
	#define HEVC_ACTION_DONE	0xff
#define HEVC_RPM_BUFFER           HEVC_ASSIST_SCRATCH_1
#define HEVC_SHORT_TERM_RPS       HEVC_ASSIST_SCRATCH_2
#define HEVC_VPS_BUFFER           HEVC_ASSIST_SCRATCH_3
#define HEVC_SPS_BUFFER           HEVC_ASSIST_SCRATCH_4
#define HEVC_PPS_BUFFER           HEVC_ASSIST_SCRATCH_5
#define HEVC_SAO_UP               HEVC_ASSIST_SCRATCH_6
#define HEVC_STREAM_SWAP_BUFFER   HEVC_ASSIST_SCRATCH_7
#define H265_MMU_MAP_BUFFER       HEVC_ASSIST_SCRATCH_7
#define HEVC_STREAM_SWAP_BUFFER2  HEVC_ASSIST_SCRATCH_8
#define HEVC_sao_mem_unit         HEVC_ASSIST_SCRATCH_9
#define HEVC_SAO_ABV              HEVC_ASSIST_SCRATCH_A
#define HEVC_sao_vb_size          HEVC_ASSIST_SCRATCH_B
#define HEVC_SAO_VB               HEVC_ASSIST_SCRATCH_C
#define HEVC_SCALELUT             HEVC_ASSIST_SCRATCH_D
#define HEVC_WAIT_FLAG            HEVC_ASSIST_SCRATCH_E
#define RPM_CMD_REG               HEVC_ASSIST_SCRATCH_F
#define LMEM_DUMP_ADR             HEVC_ASSIST_SCRATCH_F
#define DEBUG_REG1		  HEVC_ASSIST_SCRATCH_G
#define HEVC_DECODE_MODE2	  HEVC_ASSIST_SCRATCH_H
#define NAL_SEARCH_CTL            HEVC_ASSIST_SCRATCH_I
#define HEVC_DECODE_MODE	  HEVC_ASSIST_SCRATCH_J
	#define DECODE_MODE_SINGLE 0
#define DECODE_STOP_POS		  HEVC_ASSIST_SCRATCH_K
#define HEVC_AUX_ADR		  HEVC_ASSIST_SCRATCH_L
#define HEVC_AUX_DATA_SIZE	  HEVC_ASSIST_SCRATCH_M
#define HEVC_DECODE_SIZE	  HEVC_ASSIST_SCRATCH_N

#define HEVCD_MPP_ANC2AXI_TBL_DATA (0x3464 * 4)

/* HEVC Infos */
#define MAX_REF_PIC_NUM 24

#define IPP_OFFSET       0x00
#define SAO_ABV_OFFSET   (IPP_OFFSET + 0x4000)
#define SAO_VB_OFFSET    (SAO_ABV_OFFSET + 0x30000)
#define SH_TM_RPS_OFFSET (SAO_VB_OFFSET + 0x30000)
#define VPS_OFFSET       (SH_TM_RPS_OFFSET + 0x800)
#define SPS_OFFSET       (VPS_OFFSET + 0x800)
#define PPS_OFFSET       (SPS_OFFSET + 0x800)
#define SAO_UP_OFFSET    (PPS_OFFSET + 0x2000)
#define SWAP_BUF_OFFSET  (SAO_UP_OFFSET + 0x800)
#define SWAP_BUF2_OFFSET (SWAP_BUF_OFFSET + 0x800)
#define SCALELUT_OFFSET  (SWAP_BUF2_OFFSET + 0x800)
#define DBLK_PARA_OFFSET (SCALELUT_OFFSET + 0x8000)
#define DBLK_DATA_OFFSET (DBLK_PARA_OFFSET + 0x20000)
#define MMU_VBH_OFFSET   (DBLK_DATA_OFFSET + 0x40000)
#define MPRED_ABV_OFFSET (MMU_VBH_OFFSET + 0x5000)
#define MPRED_MV_OFFSET  (MPRED_ABV_OFFSET + 0x8000)
#define RPM_OFFSET       (MPRED_MV_OFFSET + 0x120000 * MAX_REF_PIC_NUM)
#define LMEM_OFFSET      (RPM_OFFSET + 0x100)

/* ISR decode status */
#define HEVC_DEC_IDLE                        0x0
#define HEVC_NAL_UNIT_VPS                    0x1
#define HEVC_NAL_UNIT_SPS                    0x2
#define HEVC_NAL_UNIT_PPS                    0x3
#define HEVC_NAL_UNIT_CODED_SLICE_SEGMENT    0x4
#define HEVC_CODED_SLICE_SEGMENT_DAT         0x5
#define HEVC_SLICE_DECODING                  0x6
#define HEVC_NAL_UNIT_SEI                    0x7
#define HEVC_SLICE_SEGMENT_DONE              0x8
#define HEVC_NAL_SEARCH_DONE                 0x9
#define HEVC_DECPIC_DATA_DONE                0xa
#define HEVC_DECPIC_DATA_ERROR               0xb
#define HEVC_SEI_DAT                         0xc
#define HEVC_SEI_DAT_DONE                    0xd

#define SIZE_WORKSPACE ALIGN(LMEM_OFFSET + 0xA00, 64 * SZ_1K)
#define SIZE_AUX (SZ_1K * 32)
#define SIZE_FRAME_MMU (0x1200 * 4)

#define PARSER_CMD_SKIP_CFG_0 0x0000090b
#define PARSER_CMD_SKIP_CFG_1 0x1b14140f
#define PARSER_CMD_SKIP_CFG_2 0x001b1910

#define AMRISC_MAIN_REQ         0x04

static const u16 parser_cmd[] = {
	0x0401,
	0x8401,
	0x0800,
	0x0402,
	0x9002,
	0x1423,
	0x8CC3,
	0x1423,
	0x8804,
	0x9825,
	0x0800,
	0x04FE,
	0x8406,
	0x8411,
	0x1800,
	0x8408,
	0x8409,
	0x8C2A,
	0x9C2B,
	0x1C00,
	0x840F,
	0x8407,
	0x8000,
	0x8408,
	0x2000,
	0xA800,
	0x8410,
	0x04DE,
	0x840C,
	0x840D,
	0xAC00,
	0xA000,
	0x08C0,
	0x08E0,
	0xA40E,
	0xFC00,
	0x7C00
};

#define RPM_BEGIN                                              0x100
#define modification_list_cur                                  0x140
#define RPM_END                                                0x180

union rpm_param {
	struct {
		unsigned short data[RPM_END - RPM_BEGIN];
	} l;
	struct {
		/* from ucode lmem, do not change this struct */
		unsigned short CUR_RPS[0x10];
		unsigned short num_ref_idx_l0_active;
		unsigned short num_ref_idx_l1_active;
		unsigned short slice_type;
		unsigned short slice_temporal_mvp_enable_flag;
		unsigned short dependent_slice_segment_flag;
		unsigned short slice_segment_address;
		unsigned short num_title_rows_minus1;
		unsigned short pic_width_in_luma_samples;
		unsigned short pic_height_in_luma_samples;
		unsigned short log2_min_coding_block_size_minus3;
		unsigned short log2_diff_max_min_coding_block_size;
		unsigned short log2_max_pic_order_cnt_lsb_minus4;
		unsigned short POClsb;
		unsigned short collocated_from_l0_flag;
		unsigned short collocated_ref_idx;
		unsigned short log2_parallel_merge_level;
		unsigned short five_minus_max_num_merge_cand;
		unsigned short sps_num_reorder_pics_0;
		unsigned short modification_flag;
		unsigned short tiles_enabled_flag;
		unsigned short num_tile_columns_minus1;
		unsigned short num_tile_rows_minus1;
		unsigned short tile_width[4];
		unsigned short tile_height[4];
		unsigned short misc_flag0;
		unsigned short pps_beta_offset_div2;
		unsigned short pps_tc_offset_div2;
		unsigned short slice_beta_offset_div2;
		unsigned short slice_tc_offset_div2;
		unsigned short pps_cb_qp_offset;
		unsigned short pps_cr_qp_offset;
		unsigned short first_slice_segment_in_pic_flag;
		unsigned short m_temporalId;
		unsigned short m_nalUnitType;
		unsigned short vui_num_units_in_tick_hi;
		unsigned short vui_num_units_in_tick_lo;
		unsigned short vui_time_scale_hi;
		unsigned short vui_time_scale_lo;
		unsigned short bit_depth;
		unsigned short profile_etc;
		unsigned short sei_frame_field_info;
		unsigned short video_signal_type;
		unsigned short modification_list[0x20];
		unsigned short conformance_window_flag;
		unsigned short conf_win_left_offset;
		unsigned short conf_win_right_offset;
		unsigned short conf_win_top_offset;
		unsigned short conf_win_bottom_offset;
		unsigned short chroma_format_idc;
		unsigned short color_description;
		unsigned short aspect_ratio_idc;
		unsigned short sar_width;
		unsigned short sar_height;
	} p;
};

enum NalUnitType {
	NAL_UNIT_CODED_SLICE_TRAIL_N = 0,	/* 0 */
	NAL_UNIT_CODED_SLICE_TRAIL_R,	/* 1 */

	NAL_UNIT_CODED_SLICE_TSA_N,	/* 2 */
	/* Current name in the spec: TSA_R */
	NAL_UNIT_CODED_SLICE_TLA,	/* 3 */

	NAL_UNIT_CODED_SLICE_STSA_N,	/* 4 */
	NAL_UNIT_CODED_SLICE_STSA_R,	/* 5 */

	NAL_UNIT_CODED_SLICE_RADL_N,	/* 6 */
	/* Current name in the spec: RADL_R */
	NAL_UNIT_CODED_SLICE_DLP,	/* 7 */

	NAL_UNIT_CODED_SLICE_RASL_N,	/* 8 */
	/* Current name in the spec: RASL_R */
	NAL_UNIT_CODED_SLICE_TFD,	/* 9 */

	NAL_UNIT_RESERVED_10,
	NAL_UNIT_RESERVED_11,
	NAL_UNIT_RESERVED_12,
	NAL_UNIT_RESERVED_13,
	NAL_UNIT_RESERVED_14,
	NAL_UNIT_RESERVED_15,

	/* Current name in the spec: BLA_W_LP */
	NAL_UNIT_CODED_SLICE_BLA,	/* 16 */
	/* Current name in the spec: BLA_W_DLP */
	NAL_UNIT_CODED_SLICE_BLANT,	/* 17 */
	NAL_UNIT_CODED_SLICE_BLA_N_LP,	/* 18 */
	/* Current name in the spec: IDR_W_DLP */
	NAL_UNIT_CODED_SLICE_IDR,	/* 19 */
	NAL_UNIT_CODED_SLICE_IDR_N_LP,	/* 20 */
	NAL_UNIT_CODED_SLICE_CRA,	/* 21 */
	NAL_UNIT_RESERVED_22,
	NAL_UNIT_RESERVED_23,

	NAL_UNIT_RESERVED_24,
	NAL_UNIT_RESERVED_25,
	NAL_UNIT_RESERVED_26,
	NAL_UNIT_RESERVED_27,
	NAL_UNIT_RESERVED_28,
	NAL_UNIT_RESERVED_29,
	NAL_UNIT_RESERVED_30,
	NAL_UNIT_RESERVED_31,

	NAL_UNIT_VPS,		/* 32 */
	NAL_UNIT_SPS,		/* 33 */
	NAL_UNIT_PPS,		/* 34 */
	NAL_UNIT_ACCESS_UNIT_DELIMITER,	/* 35 */
	NAL_UNIT_EOS,		/* 36 */
	NAL_UNIT_EOB,		/* 37 */
	NAL_UNIT_FILLER_DATA,	/* 38 */
	NAL_UNIT_SEI,		/* 39 Prefix SEI */
	NAL_UNIT_SEI_SUFFIX,	/* 40 Suffix SEI */
	NAL_UNIT_RESERVED_41,
	NAL_UNIT_RESERVED_42,
	NAL_UNIT_RESERVED_43,
	NAL_UNIT_RESERVED_44,
	NAL_UNIT_RESERVED_45,
	NAL_UNIT_RESERVED_46,
	NAL_UNIT_RESERVED_47,
	NAL_UNIT_UNSPECIFIED_48,
	NAL_UNIT_UNSPECIFIED_49,
	NAL_UNIT_UNSPECIFIED_50,
	NAL_UNIT_UNSPECIFIED_51,
	NAL_UNIT_UNSPECIFIED_52,
	NAL_UNIT_UNSPECIFIED_53,
	NAL_UNIT_UNSPECIFIED_54,
	NAL_UNIT_UNSPECIFIED_55,
	NAL_UNIT_UNSPECIFIED_56,
	NAL_UNIT_UNSPECIFIED_57,
	NAL_UNIT_UNSPECIFIED_58,
	NAL_UNIT_UNSPECIFIED_59,
	NAL_UNIT_UNSPECIFIED_60,
	NAL_UNIT_UNSPECIFIED_61,
	NAL_UNIT_UNSPECIFIED_62,
	NAL_UNIT_UNSPECIFIED_63,
	NAL_UNIT_INVALID,
};

struct codec_hevc {
	/* Buffer for the HEVC Workspace */
	void      *workspace_vaddr;
	dma_addr_t workspace_paddr;

	/* AUX buffer */
	void      *aux_vaddr;
	dma_addr_t aux_paddr;

	/* Frame MMU buffer (>= GXL) */
	void      *frame_mmu_vaddr;
	dma_addr_t frame_mmu_paddr;

	/* Contains many information parsed from the bitstream */
	union rpm_param rpm_param;

	/* Information computed from the RPM */
	u32 lcu_size; // Largest Coding Unit

	/* Current Picture Order Count */
	u32 curr_poc;

	/* ?? */
	u32 iPrevTid0POC;

	/* Housekeeping thread for marking buffers to DONE
	 * and recycling them into the hardware
	 */
	struct task_struct *buffers_thread;
};

static int codec_hevc_buffers_thread(void *data)
{
	struct vdec_session *sess = data;
	struct vdec_core *core = sess->core;

	while (!kthread_should_stop()) {
		printk("status: %08X ; level = %d ; d_si = %08X ; d_st = %08X; d_sc = %08X ; sfc = %08X\n", readl_relaxed(core->dos_base + HEVC_PARSER_INT_STATUS), readl_relaxed(core->dos_base + HEVC_STREAM_LEVEL), readl_relaxed(core->dos_base + HEVC_DECODE_SIZE), readl_relaxed(core->dos_base + DECODE_STOP_POS), readl_relaxed(core->dos_base + HEVC_STREAM_CONTROL), readl_relaxed(core->dos_base + HEVC_STREAM_FIFO_CTL));

		msleep(100);
	}

	return 0;
}

static void codec_hevc_setup_buffers(struct vdec_session *sess)
{
	int i;
	dma_addr_t buf_y_paddr = 0;
	dma_addr_t buf_uv_paddr;
	struct v4l2_m2m_buffer *buf;
	struct vdec_core *core = sess->core;
	u32 buf_size = v4l2_m2m_num_dst_bufs_ready(sess->m2m_ctx);

	/* >= GXL */
	writel_relaxed((1 << 2) | (1 << 1), core->dos_base + HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR);
	/* < GXL */
	//writel_relaxed(0, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR);

	v4l2_m2m_for_each_dst_buf(sess->m2m_ctx, buf) {
		buf_y_paddr  = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		buf_uv_paddr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 1);

		writel_relaxed(buf_y_paddr  >> 5, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_DATA);
		writel_relaxed(buf_y_paddr  >> 5, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_DATA);
		/* Double write mode ?? */
		//writel_relaxed(buf_uv_paddr >> 5, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_DATA);
	}

	/* Fill the remaining unused slots with the last buffer's Y addr ? */
	for (i = buf_size; i < MAX_REF_PIC_NUM; ++i) {
		writel_relaxed(buf_y_paddr  >> 5, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_DATA);
		//writel_relaxed(buf_uv_paddr >> 5, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_DATA);
	}

	writel_relaxed(1, core->dos_base + HEVCD_MPP_ANC2AXI_TBL_CONF_ADDR);
	writel_relaxed(1, core->dos_base + HEVCD_MPP_ANC_CANVAS_ACCCONFIG_ADDR);
	for (i = 0; i < 32; ++i)
		writel_relaxed(0, core->dos_base + HEVCD_MPP_ANC_CANVAS_DATA_ADDR);
}

static int codec_hevc_setup_workspace(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	struct codec_hevc *hevc = sess->priv;

	/* Allocate some memory for the HEVC decoder's state */
	hevc->workspace_vaddr = dma_alloc_coherent(core->dev, SIZE_WORKSPACE, &hevc->workspace_paddr, GFP_KERNEL);
	if (!hevc->workspace_vaddr) {
		printk("Failed to request HEVC Workspace\n");
		return -ENOMEM;
	}
	printk("Allocated Workspace: %08X - %08X\n", hevc->workspace_paddr, hevc->workspace_paddr + SIZE_WORKSPACE);

	hevc->frame_mmu_vaddr = dma_alloc_coherent(core->dev, SIZE_FRAME_MMU, &hevc->frame_mmu_paddr, GFP_KERNEL);
	if (!hevc->frame_mmu_vaddr) {
		printk("Failed to request HEVC frame_mmu\n");
		return -ENOMEM;
	}
	memset(hevc->frame_mmu_vaddr, 0, SIZE_FRAME_MMU);
	printk("Allocated frame_mmu: %08X - %08X\n", hevc->frame_mmu_paddr, hevc->frame_mmu_paddr + SIZE_FRAME_MMU);

	/* Setup Workspace */
	writel_relaxed(hevc->workspace_paddr + IPP_OFFSET, core->dos_base + HEVCD_IPP_LINEBUFF_BASE);
	writel_relaxed(hevc->workspace_paddr + RPM_OFFSET, core->dos_base + HEVC_RPM_BUFFER);
	writel_relaxed(hevc->workspace_paddr + SH_TM_RPS_OFFSET, core->dos_base + HEVC_SHORT_TERM_RPS);
	writel_relaxed(hevc->workspace_paddr + VPS_OFFSET, core->dos_base + HEVC_VPS_BUFFER);
	writel_relaxed(hevc->workspace_paddr + SPS_OFFSET, core->dos_base + HEVC_SPS_BUFFER);
	writel_relaxed(hevc->workspace_paddr + PPS_OFFSET, core->dos_base + HEVC_PPS_BUFFER);
	writel_relaxed(hevc->workspace_paddr + SAO_UP_OFFSET, core->dos_base + HEVC_SAO_UP);
	/* MMU */
	writel_relaxed(hevc->frame_mmu_paddr, core->dos_base + H265_MMU_MAP_BUFFER);
	/* No MMU */
	//writel_relaxed(hevc->workspace_paddr + SWAP_BUF_OFFSET, core->dos_base + HEVC_STREAM_SWAP_BUFFER);
	writel_relaxed(hevc->workspace_paddr + SWAP_BUF2_OFFSET, core->dos_base + HEVC_STREAM_SWAP_BUFFER2);
	writel_relaxed(hevc->workspace_paddr + SCALELUT_OFFSET, core->dos_base + HEVC_SCALELUT);
	writel_relaxed(hevc->workspace_paddr + DBLK_PARA_OFFSET, core->dos_base + HEVC_DBLK_CFG4);
	writel_relaxed(hevc->workspace_paddr + DBLK_DATA_OFFSET, core->dos_base + HEVC_DBLK_CFG5);

	return 0;
}

static int codec_hevc_start(struct vdec_session *sess)
{
	struct vdec_core *core = sess->core;
	struct codec_hevc *hevc;
	int ret;
	int i;

	printk("Workspace size: %u\n", SIZE_WORKSPACE);

	hevc = kzalloc(sizeof(*hevc), GFP_KERNEL);
	if (!hevc)
		return -ENOMEM;

	sess->priv = hevc;

	codec_hevc_setup_workspace(sess);

	writel_relaxed(0x5a5a55aa, core->dos_base + HEVC_PARSER_VERSION);
	writel_relaxed((1 << 14), core->dos_base + DOS_SW_RESET3);
	writel_relaxed(0, core->dos_base + HEVC_CABAC_CONTROL);
	writel_relaxed(0, core->dos_base + HEVC_PARSER_CORE_CONTROL);
	writel_relaxed(readl_relaxed(core->dos_base + HEVC_STREAM_CONTROL) | 1, core->dos_base + HEVC_STREAM_CONTROL);
	writel_relaxed(0x12345678, core->dos_base + HEVC_SHIFT_STARTCODE);
	writel_relaxed(0x9abcdef0, core->dos_base + HEVC_SHIFT_EMULATECODE);
	writel_relaxed(0x00000100, core->dos_base + HEVC_SHIFT_STARTCODE);
	writel_relaxed(0x00000300, core->dos_base + HEVC_SHIFT_EMULATECODE);
	writel_relaxed((readl_relaxed(core->dos_base + HEVC_PARSER_INT_CONTROL) & 0x03ffffff) |
			(3 << 29) | (2 << 26) | (1 << 24) | (1 << 22) | (1 << 7) | (1 << 4) | 1, core->dos_base + HEVC_PARSER_INT_CONTROL);
	writel_relaxed(readl_relaxed(core->dos_base + HEVC_SHIFT_STATUS) | (1 << 1) | 1, core->dos_base + HEVC_SHIFT_STATUS);
	writel_relaxed((3 << 6) | (2 << 4) | (2 << 1) | 1, core->dos_base + HEVC_SHIFT_CONTROL);
	writel_relaxed(1, core->dos_base + HEVC_CABAC_CONTROL);
	writel_relaxed(1, core->dos_base + HEVC_PARSER_CORE_CONTROL);
	writel_relaxed(0, core->dos_base + HEVC_DEC_STATUS_REG);

	writel_relaxed(0, core->dos_base + HEVC_IQIT_SCALELUT_WR_ADDR);
	for (i = 0; i < 1024; ++i)
		writel_relaxed(0, core->dos_base + HEVC_IQIT_SCALELUT_DATA);

	writel_relaxed(0, core->dos_base + HEVC_DECODE_SIZE);

	writel_relaxed((1 << 16), core->dos_base + HEVC_PARSER_CMD_WRITE);
	for (i = 0; i < ARRAY_SIZE(parser_cmd); ++i)
		writel_relaxed(parser_cmd[i], core->dos_base + HEVC_PARSER_CMD_WRITE);

	writel_relaxed(PARSER_CMD_SKIP_CFG_0, core->dos_base + HEVC_PARSER_CMD_SKIP_0);
	writel_relaxed(PARSER_CMD_SKIP_CFG_1, core->dos_base + HEVC_PARSER_CMD_SKIP_1);
	writel_relaxed(PARSER_CMD_SKIP_CFG_2, core->dos_base + HEVC_PARSER_CMD_SKIP_2);
	writel_relaxed((1 << 5) | (1 << 2) | 1, core->dos_base + HEVC_PARSER_IF_CONTROL);

	writel_relaxed(1, core->dos_base + HEVCD_IPP_TOP_CNTL);
	writel_relaxed((1 << 1), core->dos_base + HEVCD_IPP_TOP_CNTL);

	/* Enable NV21 reference read mode for MC */
	writel_relaxed(1 << 31, core->dos_base + HEVCD_MPP_DECOMP_CTL1);

	writel_relaxed(1, core->dos_base + HEVC_WAIT_FLAG);

	/* clear mailbox interrupt */
	writel_relaxed(1, core->dos_base + HEVC_ASSIST_MBOX1_CLR_REG);
	/* enable mailbox interrupt */
	writel_relaxed(1, core->dos_base + HEVC_ASSIST_MBOX1_MASK);
	/* disable PSCALE for hardware sharing */
	writel_relaxed(0, core->dos_base + HEVC_PSCALE_CTRL);
	/* Let the uCode do all the parsing */
	writel_relaxed(0xc, core->dos_base + NAL_SEARCH_CTL);

	/*WRITE_VREG(NAL_SEARCH_CTL,
	READ_VREG(NAL_SEARCH_CTL)
	| ((parser_sei_enable & 0x7) << 17));*/

	writel_relaxed(0, core->dos_base + DECODE_STOP_POS);
	writel_relaxed(DECODE_MODE_SINGLE, core->dos_base + HEVC_DECODE_MODE);
	writel_relaxed(0, core->dos_base + HEVC_DECODE_MODE2);

	/* AUX buffers */
	hevc->aux_vaddr = dma_alloc_coherent(core->dev, SIZE_AUX, &hevc->aux_paddr, GFP_KERNEL);
	if (!hevc->aux_vaddr) {
		printk("Failed to request HEVC AUX\n");
		return -ENOMEM;
	}
	printk("Allocated AUX: %08X - %08X\n", hevc->aux_paddr, hevc->aux_paddr + SIZE_AUX);

	writel_relaxed(hevc->aux_paddr, core->dos_base + HEVC_AUX_ADR);
	writel_relaxed((((SIZE_AUX / 2) >> 4) << 16) | ((SIZE_AUX / 2) >> 4), core->dos_base + HEVC_AUX_DATA_SIZE);

	codec_hevc_setup_buffers(sess);

	hevc->buffers_thread = kthread_run(codec_hevc_buffers_thread, sess, "buffers_done");

	printk("HEVC start OK!\n");

	return 0;
}

static int codec_hevc_stop(struct vdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	struct vdec_core *core = sess->core;

	printk("codec_hevc_stop\n");

	kthread_stop(hevc->buffers_thread);

	if (hevc->workspace_vaddr) {
		dma_free_coherent(core->dev, SIZE_WORKSPACE, hevc->workspace_vaddr, hevc->workspace_paddr);
		hevc->workspace_vaddr = 0;
	}

	if (hevc->frame_mmu_vaddr) {
		dma_free_coherent(core->dev, SIZE_FRAME_MMU, hevc->frame_mmu_vaddr, hevc->frame_mmu_paddr);
		hevc->frame_mmu_vaddr = 0;
	}

	if (hevc->aux_vaddr) {
		dma_free_coherent(core->dev, SIZE_AUX, hevc->aux_vaddr, hevc->aux_paddr);
		hevc->aux_vaddr = 0;
	}

	kfree(hevc);
	sess->priv = 0;

	return 0;
}

static void codec_hevc_prepare_new_frame(struct vdec_session *sess) {

}

static void codec_hevc_set_iPrevTid0POC(struct vdec_session *sess) {
	struct codec_hevc *hevc = sess->priv;
	u32 nal_unit_type = hevc->rpm_param.p.m_nalUnitType;
	u32 temporal_id = hevc->rpm_param.p.m_temporalId;

	if (nal_unit_type == NAL_UNIT_CODED_SLICE_IDR ||
	    nal_unit_type == NAL_UNIT_CODED_SLICE_IDR_N_LP) {
		hevc->curr_poc = 0;
		if ((temporal_id - 1) == 0)
			hevc->iPrevTid0POC = hevc->curr_poc;
	} else {
		int iMaxPOClsb =
			1 << (hevc->rpm_param.p.
			log2_max_pic_order_cnt_lsb_minus4 + 4);
		int iPrevPOClsb;
		int iPrevPOCmsb;
		int iPOCmsb;
		int iPOClsb = hevc->rpm_param.p.POClsb;

		iPrevPOClsb = hevc->iPrevTid0POC % iMaxPOClsb;
		iPrevPOCmsb = hevc->iPrevTid0POC - iPrevPOClsb;

		if ((iPOClsb < iPrevPOClsb)
			&& ((iPrevPOClsb - iPOClsb) >=
				(iMaxPOClsb / 2)))
			iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
		else if ((iPOClsb > iPrevPOClsb)
				 && ((iPOClsb - iPrevPOClsb) >
					 (iMaxPOClsb / 2)))
			iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
		else
			iPOCmsb = iPrevPOCmsb;

		if (nal_unit_type == NAL_UNIT_CODED_SLICE_BLA   ||
		    nal_unit_type == NAL_UNIT_CODED_SLICE_BLANT ||
		    nal_unit_type == NAL_UNIT_CODED_SLICE_BLA_N_LP) {
			/* For BLA picture types, POCmsb is set to 0. */
			iPOCmsb = 0;
		}
		hevc->curr_poc = (iPOCmsb + iPOClsb);
		if ((temporal_id - 1) == 0)
			hevc->iPrevTid0POC = hevc->curr_poc;
	}
}

static void codec_hevc_process_segment_header(struct vdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	u32 nal_unit_type = hevc->rpm_param.p.m_nalUnitType;
	u32 temporal_id = hevc->rpm_param.p.m_temporalId;
	u32 slice_segment_address = hevc->rpm_param.p.slice_segment_address;

	printk("nal_unit_type = %u ; temporal_id = %u ; slice_seg_addr = %u\n",
		nal_unit_type, temporal_id, slice_segment_address);

	codec_hevc_set_iPrevTid0POC(sess);

	/* ? First slice: new frame ? */
	if (slice_segment_address == 0)
		codec_hevc_prepare_new_frame(sess);
}

/* The RPM raw data isn't really usable in its state.
 * There are many hi/lo fields, others must be processed
 * to actually get the relevant information, etc.
 */
static void codec_hevc_process_rpm(struct codec_hevc *hevc)
{
	union rpm_param *rpm_param = &hevc->rpm_param;

	hevc->lcu_size = 1 << (rpm_param->p.log2_min_coding_block_size_minus3 +
		3 + rpm_param->p.log2_diff_max_min_coding_block_size);
}

/* The RPM section within the workspace contains
 * many information regarding the parsed bitstream
 */
static void codec_hevc_fetch_rpm(struct vdec_session *sess)
{
	struct codec_hevc *hevc = sess->priv;
	u16 *rpm_vaddr = hevc->workspace_vaddr + RPM_OFFSET;
	int i;

	for (i = 0; i < (RPM_END - RPM_BEGIN); i += 4) {
		int ii;
		for (ii = 0; ii < 4; ii++) {
			hevc->rpm_param.l.data[i + ii] = rpm_vaddr[i + 3 - ii];
		}
	}

	codec_hevc_process_rpm(hevc);

	printk("Size: %ux%u\n", hevc->rpm_param.p.pic_width_in_luma_samples,  hevc->rpm_param.p.pic_height_in_luma_samples);
}

static irqreturn_t codec_hevc_isr(struct vdec_session *sess)
{
	u32 dec_status;
	struct vdec_core *core = sess->core;

	dec_status = readl_relaxed(core->dos_base + HEVC_DEC_STATUS_REG);
	printk("codec_hevc_isr: %08X\n", dec_status);

	if (dec_status == HEVC_SLICE_SEGMENT_DONE) {
		writel_relaxed(readl_relaxed(core->dos_base + HEVC_WAIT_FLAG) | 2, core->dos_base + HEVC_WAIT_FLAG);
		codec_hevc_fetch_rpm(sess);
		codec_hevc_process_segment_header(sess);
		writel_relaxed(HEVC_CODED_SLICE_SEGMENT_DAT, core->dos_base + HEVC_DEC_STATUS_REG);
		writel_relaxed(AMRISC_MAIN_REQ, core->dos_base + HEVC_MCPU_INTR_REQ);
	}

	return IRQ_HANDLED;
}

struct vdec_codec_ops codec_hevc_ops = {
	.start = codec_hevc_start,
	.stop = codec_hevc_stop,
	.isr = codec_hevc_isr,
};

