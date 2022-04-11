/**
 * @file lipkg.h
 * @author LDRobot (contact@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
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

#ifndef __LIPKG_H
#define __LIPKG_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stdint.h>

#include <array>
#include <iostream>
#include <vector>

#include "pointdata.h"
#include "transform.h"

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
 public:
  LiPkg(std::string frame_id, LDVersion ld_version, bool laser_scan_dir, bool enable_angle_crop_func,
    double angle_crop_min, double angle_crop_max);
  // Lidar spin speed (Hz)
  double GetSpeed(void);
  // time stamp of the packet In milliseconds
  uint16_t GetTimestamp(void) { return timestamp_; }
  // a packet is ready
  bool IsPkgReady(void) { return is_pkg_ready_; }
  // Lidar data frame is ready
  bool IsFrameReady(void) { return is_frame_ready_; }
  void ResetFrameReady(void) { is_frame_ready_ = false; }
  // the number of errors in parser process of lidar data frame
  long GetErrorTimes(void) { return error_times_; }
  // original data package
  const std::array<PointData, POINT_PER_PACK> &GetPkgData(void);
  bool AnalysisOne(uint8_t byte);
  // parse single packet
  bool Parse(const uint8_t *data, long len);
  // combine stantard data into data frames and calibrate
  bool AssemblePacket();
  sensor_msgs::msg::LaserScan GetLaserScan() { return output_; }

  rclcpp::Clock clock;

 private:
  const int kPointFrequence = 2300;
  uint16_t timestamp_;
  double speed_;
  long error_times_;
  bool is_frame_ready_;
  bool is_pkg_ready_;
  std::string frame_id_;
  LDVersion ld_version_;
  bool laser_scan_dir_;
  bool enable_angle_crop_func_;
  double angle_crop_min_;
  double angle_crop_max_;

  LiDARFrameTypeDef pkg;
  std::array<PointData, POINT_PER_PACK> one_pkg_;
  std::vector<PointData> frame_tmp_;
  sensor_msgs::msg::LaserScan output_;

  void ToLaserscan(std::vector<PointData> src);
};


#endif  // __LIPKG_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/