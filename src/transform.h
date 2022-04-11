/**
 * @file transform.h
 * @author LDRobot (contact@ldrobot.com)
 * @brief  Ranging center conversion with left and right hand system changes App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __TRANSFORM_H
#define __TRANSFORM_H

#include <math.h>

#include <algorithm>
#include <vector>

#include "pointdata.h"

enum class LDVersion {
  LD_ZERO,     // Zero  generation lidar
  LD_THREE,    // Third generation lidar
  LD_EIGHT,    // Eight generation radar
  LD_FOURTEEN  // Fourteen generation radar
};

class SlTransform {
 private:
  bool to_right_hand_;
  double offset_x_;
  double offset_y_;
  LDVersion version_;

 public:
  SlTransform(LDVersion version, bool to_right_hand_ = false);
  Points2D Transform(const Points2D &data);
  ~SlTransform();
};

#endif  // __TRANSFORM_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/