/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "image_thumb.h"

#include <zephyr/sys/util.h>
#include "companion_proto.h"

BUILD_ASSERT(THUMB_BYTES <= COMPANION_MAX_IMAGE);

void image_thumb_from_model_input(const int8_t *model_input, size_t model_w, size_t model_h,
				  uint8_t *out, size_t out_len)
{
	const size_t plane = model_w * model_h;

	if (out_len < THUMB_BYTES || model_input == NULL || out == NULL) {
		return;
	}

	const int8_t *green = model_input + plane;

	for (size_t ty = 0; ty < THUMB_H; ty++) {
		const size_t sy = (ty * model_h) / THUMB_H;

		for (size_t tx = 0; tx < THUMB_W; tx++) {
			const size_t sx = (tx * model_w) / THUMB_W;
			const int8_t sample = green[sy * model_w + sx];

			out[ty * THUMB_W + tx] = (uint8_t)((int)sample + 128);
		}
	}
}
