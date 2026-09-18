/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#define THUMB_W 24U
#define THUMB_H 20U
#define THUMB_BYTES (THUMB_W * THUMB_H)

void image_thumb_from_model_input(const int8_t *model_input, size_t model_w, size_t model_h,
				  uint8_t *out, size_t out_len);
