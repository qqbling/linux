/*
 * Copyright (C) 2018 Maxime Jourdan <maxi.jourdan@wanadoo.fr>
 *
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

#ifndef __MESON_VDEC_PLATFORM_H_
#define __MESON_VDEC_PLATFORM_H_

#include "vdec.h"

struct vdec_format;

enum vdec_revision {
	VDEC_REVISION_GXBB,
	VDEC_REVISION_GXL,
	VDEC_REVISION_GXM,
};

struct vdec_platform {
	const struct vdec_format *formats;
	const u32 num_formats;
	enum vdec_revision revision;
};

extern const struct vdec_platform vdec_platform_gxbb;
extern const struct vdec_platform vdec_platform_gxm;
extern const struct vdec_platform vdec_platform_gxl;

#endif