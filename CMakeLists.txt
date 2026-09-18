# Copyright (c) 2026 Nordic Semiconductor
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(person_detection)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/dts)

include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)

zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)

target_sources(app PRIVATE
  src/main.c
  src/postprocessing.c
  src/usb_stream.c
  src/drivers/arducam_mega.c
  ../common/companion_proto.c
  src/image_thumb.c
  src/companion_uart.c
)
