#ifndef __MESON_VDEC_VDEC_1_H_
#define __MESON_VDEC_VDEC_1_H_

#include "vdec.h"

extern struct vdec_ops vdec_1_ops;
irqreturn_t vdec_1_isr(int irq, void *data);

#endif