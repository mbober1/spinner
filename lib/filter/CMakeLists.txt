# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SPINNER_FILTER_LOW_PASS)
  zephyr_library()
  zephyr_library_sources(low_pass_filter.c)
endif()