###################################################################################################
#
# Source and include definition
#
# Copyright (c) 2019 Packetcraft, Inc.
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
###################################################################################################

INC_DIRS += \
	$(ROOT_DIR)/ble-mesh-model/include \
	$(ROOT_DIR)/ble-mesh-model/sources/include

# Found with the command "find ble-mesh-model/sources -name '*.c'"
# run from Cordio root dir
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightlightnesssetupsr/mmdl_lightlightnesssetup_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/schedulercl/mmdl_scheduler_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genlevelsr/mmdl_gen_level_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genbatterysr/mmdl_gen_battery_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/timesr/mmdl_timesetup_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/timesr/mmdl_time_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lighthslcl/mmdl_light_hsl_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/htsr/mesh_ht_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/htsr/mesh_ht_sr_states.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lighthslsatsr/mmdl_light_hsl_sat_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genpowonoffsetupsr/mmdl_gen_powonoffsetup_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genpowerlevelcl/mmdl_gen_powerlevel_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genpowerlevelsetupsr/mmdl_gen_powerlevelsetup_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lighthslhuesr/mmdl_light_hsl_hue_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genbatterycl/mmdl_gen_battery_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genonoffsr/mmdl_gen_onoff_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genpowonoffsr/mmdl_gen_powonoff_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/gendefaulttransitioncl/mmdl_gen_default_trans_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightlightnesscl/mmdl_lightlightness_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/gendefaulttransitionsr/mmdl_gen_default_trans_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightctlcl/mmdl_light_ctl_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genpowonoffcl/mmdl_gen_powonoff_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/htcl/mesh_ht_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/bindings/mmdl_bindings_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightctlsr/mmdl_light_ctl_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightctlsr/mmdl_light_ctl_temp_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightctlsr/mmdl_light_ctl_setup_sr.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lighthslsr/mmdl_light_hsl_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lighthslsr/mmdl_light_hsl_setup_sr.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genlevelcl/mmdl_gen_level_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/common/mmdl_common_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/scenesr/mmdl_scene_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/scenesr/mmdl_scene_setup_sr.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genonoffcl/mmdl_gen_onoff_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/scenecl/mmdl_scene_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/lightlightnesssr/mmdl_lightlightness_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/timecl/mmdl_time_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/vendortestcl/mmdl_vendor_test_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/genpowerlevelsr/mmdl_gen_powerlevel_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/schedulersr/mmdl_scheduler_setup_sr.c
C_FILES += $(ROOT_DIR)/ble-mesh-model/sources/schedulersr/mmdl_scheduler_sr_main.c