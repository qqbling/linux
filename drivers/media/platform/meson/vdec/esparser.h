#ifndef __MESON_VDEC_ESPARSER_H_
#define __MESON_VDEC_ESPARSER_H_

#include "vdec.h"

int esparser_init(struct platform_device *pdev, struct vdec_core *core);
int esparser_power_up(struct vdec_core *core);
int esparser_process_buf(struct vdec_core *core, struct vb2_v4l2_buffer *vbuf);
int stbuf_power_up(struct vdec_core *core);

#endif