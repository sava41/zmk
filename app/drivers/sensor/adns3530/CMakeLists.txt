# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)
zephyr_include_directories_ifdef(CONFIG_ADNS3530)

zephyr_library()

zephyr_library_sources(adns3530.c)
zephyr_library_sources_ifdef(CONFIG_ADNS3530_TRIGGER adns3530_trigger.c)
