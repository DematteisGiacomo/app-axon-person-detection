/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "companion_uart.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "companion_proto.h"
#include "image_thumb.h"

LOG_MODULE_REGISTER(companion_uart, LOG_LEVEL_INF);

#if IS_ENABLED(CONFIG_COMPANION_LINK)

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(companion_uart));

static int64_t last_send_ms;

static struct companion_detect_image enc_msg;
static uint8_t wire_buf[COMPANION_FRAME_MAX];

static uint16_t top_score_mille(const struct detection_box *boxes, size_t box_count)
{
	uint16_t best = 0;

	for (size_t i = 0; i < box_count; i++) {
		const uint32_t mille = (uint32_t)(boxes[i].score * 1000.f);

		if (mille > best) {
			best = (uint16_t)MIN(mille, 1000U);
		}
	}

	return best;
}

int companion_uart_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("Companion UART not ready");
		return -ENODEV;
	}

	last_send_ms = 0;
	LOG_INF("Companion UART ready");

	return 0;
}

int companion_uart_send_detection(uint32_t frame_id, const struct detection_box *boxes,
				  size_t box_count, const uint8_t *thumb, size_t thumb_len)
{
	size_t wire_len;
	int err;

	if (box_count == 0 || thumb == NULL || thumb_len == 0 || thumb_len > COMPANION_MAX_IMAGE) {
		return -EINVAL;
	}

	const int64_t now = k_uptime_get();
	const int64_t min_interval_ms = (int64_t)CONFIG_COMPANION_MIN_INTERVAL_SEC * 1000;

	if (last_send_ms != 0 && (now - last_send_ms) < min_interval_ms) {
		LOG_DBG("Companion debounced");
		return 1;
	}

	memset(&enc_msg, 0, sizeof(enc_msg));
	enc_msg.hdr.magic = COMPANION_MAGIC;
	enc_msg.hdr.version = COMPANION_VERSION;
	enc_msg.hdr.msg_type = COMPANION_MSG_DETECT_IMAGE;
	enc_msg.hdr.frame_id = frame_id;
	enc_msg.hdr.detect_count = (uint8_t)MIN(box_count, 255U);
	enc_msg.hdr.top_score_mille = top_score_mille(boxes, box_count);
	enc_msg.hdr.image_len = (uint16_t)thumb_len;
	enc_msg.hdr.image_fmt = COMPANION_FMT_RAW_LUMA;
	enc_msg.hdr.thumb_w = THUMB_W;
	enc_msg.hdr.thumb_h = THUMB_H;
	memcpy(enc_msg.image, thumb, thumb_len);

	err = companion_frame_encode(&enc_msg, wire_buf, sizeof(wire_buf), &wire_len);
	if (err) {
		return err;
	}

	for (size_t off = 0; off < wire_len; off++) {
		uart_poll_out(uart_dev, wire_buf[off]);
	}

	last_send_ms = now;
	LOG_INF("Companion sent frame %u (%u bytes, score %u)", frame_id, (unsigned)wire_len,
		enc_msg.hdr.top_score_mille);

	return 0;
}

#else

int companion_uart_init(void)
{
	return 0;
}

int companion_uart_send_detection(uint32_t frame_id, const struct detection_box *boxes,
				  size_t box_count, const uint8_t *thumb, size_t thumb_len)
{
	ARG_UNUSED(frame_id);
	ARG_UNUSED(boxes);
	ARG_UNUSED(box_count);
	ARG_UNUSED(thumb);
	ARG_UNUSED(thumb_len);

	return -ENOTSUP;
}

#endif
