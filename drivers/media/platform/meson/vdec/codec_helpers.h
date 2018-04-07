#ifndef __MESON_VDEC_CODEC_HELPERS_H_
#define __MESON_VDEC_CODEC_HELPERS_H_

#include "vdec.h"

void codec_helper_set_canvases(struct vdec_session *sess, void *reg_base);
void codec_helper_fill_buf_idx(struct vdec_session *sess, u32 buffer_index);

#endif