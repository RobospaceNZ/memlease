#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if(CONFIG_MEMLEASE)
  target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/memlease.c)
endif()
