/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <assert.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <axon/nrf_axon_platform.h>
#include <drivers/axon/nrf_axon_driver.h>
#include <drivers/axon/nrf_axon_nn_infer.h>

#include "generated/nrf_axon_model_person_det_.h"
#include "companion_uart.h"
#include "image_thumb.h"
#include "postprocessing.h"
#include "usb_stream.h"

LOG_MODULE_REGISTER(main);

#include <hal/nrf_gpio.h>
#define TRACE_PIN_CAPTURE NRF_GPIO_PIN_MAP(1, 10)
#define TRACE_PIN_PRE	  NRF_GPIO_PIN_MAP(1, 11)
#define TRACE_PIN_INFER	  NRF_GPIO_PIN_MAP(1, 12)
#define TRACE_PIN_POST	  NRF_GPIO_PIN_MAP(1, 13)

#define CAM_WIDTH    128
#define CAM_HEIGHT   128
#define MODEL_WIDTH  160
#define MODEL_HEIGHT 128
#define PAD_LEFT     (((MODEL_WIDTH) - (CAM_WIDTH)) / 2)
#define PAD_TOP	     (((MODEL_HEIGHT) - (CAM_HEIGHT)) / 2)

#define FRAME_RGB565_BYTES ((CAM_WIDTH) * (CAM_HEIGHT) * 2)

#define LUT_SIZE_5_BITS 32
#define LUT_SIZE_6_BITS 64

#define MAX_BOXES_LOG 8

static int8_t input_buf[MODEL_WIDTH * MODEL_HEIGHT * 3];
static int8_t output_buf[NRF_AXON_MODEL_PERSON_DET_PACKED_OUTPUT_SIZE];

static int8_t lut_red_blue[LUT_SIZE_5_BITS];
static int8_t lut_green[LUT_SIZE_6_BITS];

static uint32_t frame_id;

static uint8_t detect_thumb[THUMB_BYTES];

static const struct gpio_dt_spec led_capture = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_detection = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

static int led_init(const struct gpio_dt_spec *spec)
{
	int err;

	if (!gpio_is_ready_dt(spec)) {
		LOG_ERR("GPIO %s is not ready", spec->port->name);
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("Failed to configure %s pin %u (err %d)", spec->port->name, spec->pin, err);
		return err;
	}

	return 0;
}

static inline int8_t quantize(const float value, const nrf_axon_nn_compiled_model_input_s *in)
{
	const uint32_t quant_mult = in->quant_mult;
	const uint8_t quant_round = in->quant_round;
	const int8_t quant_zp = in->quant_zp;

	const float scale = (float)quant_mult / (float)(1 << quant_round);
	const int32_t quantized = (int32_t)(value * scale) + quant_zp;

	return (int8_t)__ssat(quantized, 8);
}

static void prefill_input_buf(const nrf_axon_nn_compiled_model_input_s *in)
{
	const float gray_symmetric = 0.0f;
	const int8_t gray = quantize(gray_symmetric, in);

	memset(input_buf, gray, sizeof(input_buf));
}

static void prefill_luts(const nrf_axon_nn_compiled_model_input_s *in)
{
	for (size_t i = 0; i < ARRAY_SIZE(lut_red_blue); i++) {
		const float value = (float)i / 32.f;
		const float value_sym = (value * 2.f) - 1.f;

		lut_red_blue[i] = quantize(value_sym, in);
	}

	for (size_t i = 0; i < ARRAY_SIZE(lut_green); i++) {
		const float value = (float)i / 64.f;
		const float value_sym = (value * 2.f) - 1.f;

		lut_green[i] = quantize(value_sym, in);
	}
}

static inline uint16_t extract_pixel(const uint8_t *data, const size_t pixel)
{
	const size_t offset = pixel * 2;

	return (uint16_t)((uint16_t)data[offset] << 8 | data[offset + 1]);
}

static void convert_chunk_to_model_input(const uint8_t *chunk_buf, const size_t pixel_start,
					 const size_t pixel_count)
{
	for (size_t p = 0; p < pixel_count; p++) {
		const size_t pixel_idx = pixel_start + p;
		const size_t cam_row = pixel_idx / CAM_WIDTH;
		const size_t cam_col = pixel_idx % CAM_WIDTH;
		const size_t model_row = PAD_TOP + cam_row;
		const size_t model_col = PAD_LEFT + cam_col;
		const size_t dst_offset = model_row * MODEL_WIDTH + model_col;

		const uint16_t pixel = extract_pixel(chunk_buf, p);
		const uint8_t r5 = (pixel >> 11) & 0x1f;
		const uint8_t g6 = (pixel >> 5) & 0x3f;
		const uint8_t b5 = pixel & 0x1f;

		input_buf[0 * MODEL_WIDTH * MODEL_HEIGHT + dst_offset] = (int8_t)lut_red_blue[r5];
		input_buf[1 * MODEL_WIDTH * MODEL_HEIGHT + dst_offset] = (int8_t)lut_green[g6];
		input_buf[2 * MODEL_WIDTH * MODEL_HEIGHT + dst_offset] = (int8_t)lut_red_blue[b5];
	}
}

static int capture_one_frame(const struct device *video)
{
	size_t total = 0;

	usb_stream_frame_begin(frame_id, CAM_WIDTH, CAM_HEIGHT, FRAME_RGB565_BYTES);
	while (total < FRAME_RGB565_BYTES) {
		struct video_buffer *vbuf;
		int err = video_dequeue(video, &vbuf, K_FOREVER);

		if (err == -EAGAIN) {
			continue;
		}
		if (err) {
			LOG_ERR("video_dequeue failed: %d", err);
			return err;
		}

		const size_t room = FRAME_RGB565_BYTES - total;
		const size_t chunk = min(vbuf->bytesused, room);

		nrf_gpio_pin_set(TRACE_PIN_PRE);
		convert_chunk_to_model_input(vbuf->buffer, total / 2, chunk / 2);
		usb_stream_frame_chunk(vbuf->buffer, chunk);
		nrf_gpio_pin_clear(TRACE_PIN_PRE);


		total += chunk;

		vbuf->type = VIDEO_BUF_TYPE_OUTPUT;
		video_enqueue(video, vbuf);
	}

	usb_stream_frame_end();

	return 0;
}

static void log_bounding_boxes(const struct detection_box *boxes, const size_t n)
{
	for (size_t i = 0; i < n; i++) {
		LOG_INF("Bounding box %d: head %s, [%.1f, %.1f, %.1f, %.1f] score %.3f", i,
			model_head_name(boxes[i].head_id), (double)boxes[i].x1, (double)boxes[i].y1,
			(double)boxes[i].x2, (double)boxes[i].y2, (double)boxes[i].score);
	}
}

static int capture_and_detect(const struct device *video, const nrf_axon_nn_compiled_model_s *model)
{
	int err;
	nrf_axon_result_e result;

	struct detection_box boxes[MAX_BOXES_LOG];

	(void)gpio_pin_toggle_dt(&led_capture);

	nrf_gpio_pin_set(TRACE_PIN_CAPTURE);
	err = capture_one_frame(video);
	if (err) {
		LOG_ERR("Failed to capture frame (err %d)", err);
		(void)video_stream_stop(video, VIDEO_BUF_TYPE_OUTPUT);
		return -1;
	}
	nrf_gpio_pin_clear(TRACE_PIN_CAPTURE);

	nrf_gpio_pin_set(TRACE_PIN_INFER);
	result = nrf_axon_nn_model_infer_sync(model, input_buf, output_buf);
	nrf_gpio_pin_clear(TRACE_PIN_INFER);
	if (result != NRF_AXON_RESULT_SUCCESS) {
		LOG_ERR("Inference failed (result %d)", result);
		(void)gpio_pin_set_dt(&led_detection, 0);
		return -1;
	}

	nrf_gpio_pin_set(TRACE_PIN_POST);
	const size_t n = decode_output(model, output_buf, boxes, MAX_BOXES_LOG);
	nrf_gpio_pin_clear(TRACE_PIN_POST);

	usb_stream_send_detections(frame_id, MODEL_WIDTH, MODEL_HEIGHT,
				   PAD_LEFT, PAD_TOP, boxes, n);

	if (n > 0) {
		image_thumb_from_model_input(input_buf, MODEL_WIDTH, MODEL_HEIGHT, detect_thumb,
					     sizeof(detect_thumb));
		(void)companion_uart_send_detection(frame_id, boxes, n, detect_thumb,
						    sizeof(detect_thumb));

		(void)gpio_pin_set_dt(&led_detection, 1);
		log_bounding_boxes(boxes, n);
	} else {
		(void)gpio_pin_set_dt(&led_detection, 0);
		LOG_INF("No detections");
	}

	return 0;
}

int main(void)
{
	int err;

	nrf_axon_result_e result;
	const nrf_axon_nn_compiled_model_s *model = &model_person_det;
	const nrf_axon_nn_compiled_model_input_s *model_inputs =
		nrf_axon_nn_model_1st_external_input(model);

	const struct device *video = DEVICE_DT_GET(DT_NODELABEL(arducam_mega));
	struct video_buffer *vbuf;
	struct video_format fmt = {.type = VIDEO_BUF_TYPE_INPUT,
				   .pixelformat = VIDEO_PIX_FMT_RGB565,
				   .width = CAM_WIDTH,
				   .height = CAM_HEIGHT,
				   .pitch = CAM_WIDTH * 2};

	if (led_init(&led_capture) != 0) {
		return -1;
	}
	if (led_init(&led_detection) != 0) {
		return -1;
	}

	if (!device_is_ready(video)) {
		LOG_ERR("Video device not ready");
		return -1;
	}

	err = video_set_format(video, &fmt);
	if (err) {
		LOG_ERR("Setting video format failed (err %d)", err);
		return -1;
	}

	for (size_t i = 0; i < CONFIG_VIDEO_BUFFER_POOL_NUM_MAX; i++) {
		vbuf = video_buffer_alloc(1024, K_NO_WAIT);
		if (vbuf == NULL) {
			LOG_ERR("Allocation failed for video buffer %u", i);
			return -1;
		}
		vbuf->type = VIDEO_BUF_TYPE_OUTPUT;
		video_enqueue(video, vbuf);
	}

	result = nrf_axon_platform_init();
	if (result != NRF_AXON_RESULT_SUCCESS) {
		LOG_ERR("Axon platform init failed (result %d)", result);
		return -1;
	}

	result = nrf_axon_nn_model_validate(model);
	if (result != NRF_AXON_RESULT_SUCCESS) {
		LOG_ERR("Model validation failed (result %d)", result);
		return -1;
	}

	prefill_input_buf(model_inputs);
	prefill_luts(model_inputs);
	decode_init(model);

	err = usb_stream_init();
	if (err != 0) {
		LOG_WRN("USB stream init failed (err %d), continuing without streaming", err);
	}

	err = companion_uart_init();
	if (err != 0) {
		LOG_WRN("Companion UART init failed (err %d)", err);
	}

	nrf_gpio_cfg_output(TRACE_PIN_CAPTURE);
	nrf_gpio_cfg_output(TRACE_PIN_PRE);
	nrf_gpio_cfg_output(TRACE_PIN_INFER);
	nrf_gpio_cfg_output(TRACE_PIN_POST);

	LOG_INF("Person detection start");

	err = video_stream_start(video, VIDEO_BUF_TYPE_OUTPUT);
	if (err) {
		LOG_ERR("Failed to start stream (err %d)", err);
		return -1;
	}
	while (true) {
		err = capture_and_detect(video, model);
		if (err) {
			return -1;
		}

		frame_id++;
	}

	return 0;
}
