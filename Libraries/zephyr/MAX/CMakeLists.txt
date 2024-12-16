##############################################################################
#
# Copyright (C) 2023-2024 Analog Devices, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
##############################################################################


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
    ./Source/${TARGET_UC}
    ${MSDK_LIBRARY_DIR}/CMSIS/Include
    ${MSDK_CMSIS_DIR}/Include
    ${MSDK_PERIPH_INC_DIR}
)

if (CONFIG_UDC_MAX32)
    zephyr_compile_definitions(
        -DUSE_ZEPHYR_USB_STACK=1
    )

    set(MSDK_MAXUSB_DIR ${MSDK_LIBRARY_DIR}/MAXUSB)

    zephyr_include_directories(
        ${MSDK_MAXUSB_DIR}/include/core
        ${MSDK_MAXUSB_DIR}/include/core/musbhsfc
    )

    zephyr_library_sources(
        ${MSDK_MAXUSB_DIR}/src/core/usb_event.c
        ${MSDK_MAXUSB_DIR}/src/core/musbhsfc/usb.c
    )
endif()

add_subdirectory_ifdef(CONFIG_SOC_MAX32520 Source/MAX32520)
add_subdirectory_ifdef(CONFIG_SOC_MAX32570 Source/MAX32570)
add_subdirectory_ifdef(CONFIG_SOC_MAX32572 Source/MAX32572)
add_subdirectory_ifdef(CONFIG_SOC_MAX32650 Source/MAX32650)
add_subdirectory_ifdef(CONFIG_SOC_MAX32655 Source/MAX32655)
add_subdirectory_ifdef(CONFIG_SOC_MAX32657 Source/MAX32657)
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
