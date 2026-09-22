# Copyright (c) 2026 Gloppy16
# SPDX-License-Identifier: GPL-2.0-or-later

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ADNS5050 src/adns5050.c)
