# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SPINNER_FEEDBACK_RTIO rtio_feedback.c rtio_feedback_driver.c)
zephyr_library_sources_ifdef(CONFIG_SPINNER_FEEDBACK_HALLS_STM32 halls_stm32.c)
