# Copyright (c) 2026 Gloppy16
# SPDX-License-Identifier: GPL-2.0-or-later

if(CONFIG_ADNS5050)
	zephyr_library()
	zephyr_library_sources(src/adns5050.c)
endif()
