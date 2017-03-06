#
# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
ifeq ($(BOARD_HAS_ASACZ),true)
LOCAL_PATH := $(my-dir)
include $(CLEAR_VARS)

#
# ========================================================
#
# Static library for target
#
# ========================================================
#
APP_STL := stlport_static
LOCAL_CFLAGS += -DOLINUXINO_LIB -DANDROID
LOCAL_C_INCLUDES := 	$(LOCAL_PATH)/inc \
			$(LOCAL_PATH)/lib_inc
LOCAL_MODULE := libCC2650_fw_update_Android
LOCAL_STATIC_LIBRARIES += libstdc++
LOCAL_SRC_FILES := src/sblAppEx.cpp src/sbl_device.cpp src/sbl_device_cc2650.cpp src/sbllib.cpp ../OLinuxino_RF_data_send_receive/radio_asac_barebone.c
include $(BUILD_STATIC_LIBRARY)

endif

