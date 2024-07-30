/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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

#ifndef _RECORD_H_
#define _RECORD_H_

#define EMBEDDING_SIZE 64

#define SHOW_START_X (TFT_WIDTH - HEIGHT_ID) / 2
#define SHOW_START_Y (TFT_HEIGHT - WIDTH_ID) / 2

int record(void);
void show_keyboard(void);
void init_cnn_from_flash(void);

#endif // _RECORD_H_
