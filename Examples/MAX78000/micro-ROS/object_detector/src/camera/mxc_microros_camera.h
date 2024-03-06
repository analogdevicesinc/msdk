/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

#include "mxc_microros.h"
#include "mxc_errors.h"

#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/region_of_interest.h>
#include <geometry_msgs/msg/polygon_stamped.h>

int mxc_microros_camera_init(unsigned int width, unsigned int height, const char* encoding);
int mxc_microros_camera_capture(sensor_msgs__msg__Image *out_img);
int mxc_microros_camera_run_cnn(sensor_msgs__msg__Image* input_image, sensor_msgs__msg__RegionOfInterest *output_roi, geometry_msgs__msg__PolygonStamped *output_polygon);