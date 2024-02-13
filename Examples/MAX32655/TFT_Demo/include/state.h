/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#ifndef EXAMPLES_MAX32655_TFT_DEMO_INCLUDE_STATE_H_
#define EXAMPLES_MAX32655_TFT_DEMO_INCLUDE_STATE_H_

typedef int (*Init_func)(void);
typedef int (*Keypad_process)(int key);
typedef void (*Time_Tick)(void);

typedef struct _State {
    char *name;
    Init_func init;
    Keypad_process prcss_key;
    Time_Tick tick;
    unsigned int timeout;
} State;

void state_init(void);

State *state_get_current(void);
int state_set_current(State *state);

State *get_home_state(void);
State *get_keypad_state(void);
State *get_info_state(void);

#endif // EXAMPLES_MAX32655_TFT_DEMO_INCLUDE_STATE_H_
