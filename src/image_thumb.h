/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <zephyr/autoconf.h>

#define THUMB_W ((uint16_t)CONFIG_COMPANION_THUMB_W)
#define THUMB_H ((uint16_t)CONFIG_COMPANION_THUMB_H)
#define THUMB_BYTES CONFIG_COMPANION_THUMB_BYTES

void image_thumb_from_model_input(const int8_t *model_input, size_t model_w, size_t model_h,
				  uint8_t *out, size_t out_len);
