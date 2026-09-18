/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#pragma once

#include <stdint.h>

#include "postprocessing.h"

int companion_uart_init(void);

int companion_uart_send_detection(uint32_t frame_id, const struct detection_box *boxes,
				  size_t box_count, const uint8_t *thumb, size_t thumb_len);
