#ifndef __MESON_VDEC_ESPARSER_H_
#define __MESON_VDEC_ESPARSER_H_

#include "vdec.h"

int esparser_init(struct platform_device *pdev, struct vdec_core *core);
int esparser_power_up(struct vdec_session *sess);
void esparser_queue_all_src(struct work_struct *work);

#endif