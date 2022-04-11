/**
 * @file main.cpp
 * @author LDRobot (contact@ldrobot.com)
 * @brief  main process App
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
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

#include <iostream>

#include "cmd_interface_linux.h"
#include "lipkg.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // create a ROS2 Node
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); 

  std::string product_name;
	std::string topic_name;
	std::string port_name;
	std::string frame_id;
  bool laser_scan_dir = true;
  bool enable_angle_crop_func = false;
  double angle_crop_min = 0.0;
  double angle_crop_max = 0.0;

  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<std::string>("frame_id", frame_id);
  node->declare_parameter<bool>("laser_scan_dir", laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", angle_crop_max);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("port_name", port_name);
  node->get_parameter("frame_id", frame_id);
  node->get_parameter("laser_scan_dir", laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", enable_angle_crop_func);
  node->get_parameter("angle_crop_min", angle_crop_min);
  node->get_parameter("angle_crop_max", angle_crop_max);

  RCLCPP_INFO(node->get_logger(), " [ldrobot] SDK Pack Version is v2.1.4");
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <topic_name>: %s ,<port_name>: %s ,<frame_id>: %s", 
              topic_name.c_str(), port_name.c_str(), frame_id.c_str());

  RCLCPP_INFO(node->get_logger(), "[ldrobot] <laser_scan_dir>: %s,<enable_angle_crop_func>: %s,<angle_crop_min>: %f,<angle_crop_max>: %f",
   (laser_scan_dir?"Counterclockwise":"Clockwise"), (enable_angle_crop_func?"true":"false"), angle_crop_min, angle_crop_max);

  LiPkg *lidar_pkg = nullptr;
  uint32_t baudrate = 0;
 
  if(product_name == "LDLiDAR_LD14") {
    baudrate = 115200;
    lidar_pkg = new LiPkg(frame_id, LDVersion::LD_FOURTEEN, laser_scan_dir, 
      enable_angle_crop_func, angle_crop_min, angle_crop_max);
  } else{
    RCLCPP_ERROR(node->get_logger()," [ldrobot] Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  if (topic_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] fail, topic_name is empty!");
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

  if (port_name.empty() == false) {
    CmdInterfaceLinux cmd_port(baudrate);

    cmd_port.SetReadCallback([&lidar_pkg](const char *byte, size_t len) {
      if (lidar_pkg->Parse((uint8_t *)byte, len)) {
        lidar_pkg->AssemblePacket();
      }
    });

    if (cmd_port.Open(port_name)) {
      RCLCPP_INFO(node->get_logger(), " [ldrobot] open %s device %s success!", product_name.c_str(), port_name.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), " [ldrobot] open %s device %s fail!", product_name.c_str(), port_name.c_str());
      exit(EXIT_FAILURE);
    }

    rclcpp::WallRate r(6); //Hz

    while (rclcpp::ok()) {
      if (lidar_pkg->IsFrameReady()) {
        publisher->publish(lidar_pkg->GetLaserScan());
        lidar_pkg->ResetFrameReady();
#if 1
        sensor_msgs::msg::LaserScan data = lidar_pkg->GetLaserScan();
        unsigned int lens = data.ranges.size();
        RCLCPP_INFO_STREAM(node->get_logger(), "current_speed(hz): " << lidar_pkg->GetSpeed() << " "
                  << "len: " << lens << " "
                  << "angle_min: " << RADIAN_TO_ANGLE(data.angle_min) << " "
                  << "angle_max: " << RADIAN_TO_ANGLE(data.angle_max)); 
        RCLCPP_INFO_STREAM(node->get_logger(), "----------------------------");
        for (unsigned int i = 0; i < lens; i++) {
          float angle_n = RADIAN_TO_ANGLE(data.angle_min + i * data.angle_increment);
          RCLCPP_INFO_STREAM(node->get_logger(), "angle: " << angle_n << " "
                    << "range(m): " << data.ranges[i] << " "
                    << "intensites: " << data.intensities[i]);
        }
#endif
      }
      r.sleep();
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] fail, port_name is empty!");
    exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(node->get_logger(), "this node of ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
