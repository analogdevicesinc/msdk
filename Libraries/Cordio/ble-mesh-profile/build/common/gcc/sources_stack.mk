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
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/cfg \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/access \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/api \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/bearer \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/friend \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/include \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/local_config \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/lpn \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/network \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/security \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/seq_manager \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/services/timers \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/services/utils \
	$(ROOT_DIR)/ble-mesh-profile/sources/stack/transports

# Found with the command "find ble-mesh-profile/sources/stack -name '*.c'"
# run from Cordio root dir
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/services/utils/mesh_utils.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/friend/mesh_friend_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/friend/mesh_friend_data.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/friend/mesh_friend_queue.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/friend/mesh_friend_sm.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/friend/mesh_friend_act.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/lpn/mesh_lpn_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/lpn/mesh_lpn_act.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/lpn/mesh_lpn_sm.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/seq_manager/mesh_seq_manager.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/bearer/mesh_gatt_bearer.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/bearer/mesh_bearer.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/bearer/mesh_adv_bearer.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/security/mesh_security_crypto_nwk.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/security/mesh_security_crypto_utr.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/security/mesh_security_toolbox.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/security/mesh_security_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/security/mesh_security_deriv.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/security/mesh_security_crypto_beacon.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_upper_transport.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_sar_rx_history.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_sar_tx.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_upper_transport_heartbeat.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_lower_transport.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_sar_rx.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/transports/mesh_replay_protection.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/local_config/mesh_local_config.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/cfg/cfg_mesh_stack.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/proxy/mesh_proxy_sr.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/proxy/mesh_proxy_cl.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/proxy/mesh_proxy_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_api.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_app_key.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_cl_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_app_net_key.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_messages.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_api.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_pub_subscr.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_cl_api.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_sr_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/api/mesh_cfg_mdl_opcodes.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/access/mesh_access_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/access/mesh_access_period_pub.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/network/mesh_network_cache.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/network/mesh_network_mgmt.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/network/mesh_network_if.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/network/mesh_network_main.c
C_FILES += $(ROOT_DIR)/ble-mesh-profile/sources/stack/network/mesh_network_beacon.c
