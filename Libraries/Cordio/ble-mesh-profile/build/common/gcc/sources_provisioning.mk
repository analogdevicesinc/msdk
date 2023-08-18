###################################################################################################
#
# Source and include definition
#
# Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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
	$(ROOT_DIR)/ble-mesh-profile/include \
	$(ROOT_DIR)/ble-mesh-profile/sources/provisioning

# Found with the command "find ble-mesh-profile/sources/provisioning -name '*.c'"
# run from Cordio root directory
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_sr_sm.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_cl_act.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_common.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_beacon.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_cl_sm.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_br_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/provisioning/mesh_prv_sr_act.c
