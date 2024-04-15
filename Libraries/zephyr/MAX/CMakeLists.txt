# Makefile - Analog Devices SDK
#
# Copyright (c) 2023 Analog Devices, Inc.
#
# SPDX-License-Identifier: Apache-2.0


if (${CONFIG_SOC} MATCHES "max32666")
    set(TARGET_LC "max32665")
elseif (${CONFIG_SOC} MATCHES "max326651")
    set(TARGET_LC "max32650")
else ()
    set(TARGET_LC ${CONFIG_SOC})
endif()
string(TOUPPER ${TARGET_LC} TARGET_UC)

zephyr_compile_definitions(
    -DTARGET=${TARGET_UC}
    -DMSDK_NO_GPIO_CLK_INIT=1
    -DMSDK_NO_LOCKING=1
)

get_filename_component(MSDK_LIBRARY_DIR "./Libraries" ABSOLUTE)
set(MSDK_CMSIS_DIR       ${MSDK_LIBRARY_DIR}/CMSIS/Device/Maxim/${TARGET_UC})
set(MSDK_PERIPH_DIR      ${MSDK_LIBRARY_DIR}/PeriphDrivers)
set(MSDK_PERIPH_SRC_DIR  ${MSDK_PERIPH_DIR}/Source)
set(MSDK_PERIPH_INC_DIR  ${MSDK_PERIPH_DIR}/Include/${TARGET_UC})

zephyr_include_directories(
    ./Include
    ${MSDK_CMSIS_DIR}/Include
    ${MSDK_PERIPH_INC_DIR}
)

add_subdirectory_ifdef(CONFIG_SOC_MAX32520 Source/MAX32520)
add_subdirectory_ifdef(CONFIG_SOC_MAX32570 Source/MAX32570)
add_subdirectory_ifdef(CONFIG_SOC_MAX32572 Source/MAX32572)
add_subdirectory_ifdef(CONFIG_SOC_MAX32650 Source/MAX32650)
add_subdirectory_ifdef(CONFIG_SOC_MAX32655 Source/MAX32655)
add_subdirectory_ifdef(CONFIG_SOC_MAX32660 Source/MAX32660)
add_subdirectory_ifdef(CONFIG_SOC_MAX32662 Source/MAX32662)
add_subdirectory_ifdef(CONFIG_SOC_MAX32665 Source/MAX32665)
add_subdirectory_ifdef(CONFIG_SOC_MAX32666 Source/MAX32665)
add_subdirectory_ifdef(CONFIG_SOC_MAX32670 Source/MAX32670)
add_subdirectory_ifdef(CONFIG_SOC_MAX32672 Source/MAX32672)
add_subdirectory_ifdef(CONFIG_SOC_MAX32675 Source/MAX32675)
add_subdirectory_ifdef(CONFIG_SOC_MAX32680 Source/MAX32680)
add_subdirectory_ifdef(CONFIG_SOC_MAX32690 Source/MAX32690)
add_subdirectory_ifdef(CONFIG_SOC_MAX78000 Source/MAX78000)
add_subdirectory_ifdef(CONFIG_SOC_MAX78002 Source/MAX78002)
