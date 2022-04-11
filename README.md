- [cn](#操作指南)
- [en](#Instructions)
# 操作指南

>此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品，产品型号为:
> - LDROBOT LiDAR LD14

## 0. 获取雷达的ROS2功能包
```bash
$ cd ~

$ mkdir -p ldlidar_ros2_ws/src

$ cd ldlidar_ros2_ws/src

$ git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
# 或者
$ git clone  https://gitee.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
```
## 1. 系统设置
- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式使雷达连接到你的系统主板.
- 第二步，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
$ cd ~/ldlidar_ros2_ws

$ sudo chmod 777 /dev/ttyUSB0
```
- 第三步，修改`launch/`目录下雷达产品型号对应的lanuch文件中的`port_name`值，以ld14.launch.py为例，如下所示.

```py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'topic_name': 'scan'},
        {'port_name': '/dev/ttyUSB0'},
        {'frame_id': 'base_laser'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
---
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  return LaunchDescription([
    Node(
      # LDROBOT LiDAR publisher node
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'topic_name': 'scan'},
        {'port_name': '/dev/ttyUSB0'},
        {'frame_id': 'base_laser'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
    )
  ])
```
## 2. 编译方法

使用colcon编译.

```bash
$ cd ~/ldlidar_ros2_ws

$ colcon build
```
## 3. 运行方法

```bash
$ source install/setup.bash
```
- 产品型号为 LDROBOT LiDAR LD14

  ``` bash
  $ ros2 launch ldlidar_sl_ros2 ld14.launch.py
  ```
##   4. 测试

> 代码支持ubuntu20.04 ROS2 foxy版本及以上测试，使用rviz2可视化。
- 新打开一个终端 (Ctrl + Alt + T),运行命令:`rviz2`,并通过Rviz2工具打开readme文件所在目录的rviz2文件夹下面的ldlidar.rviz文件
```bash
$ rviz2
```

| 产品型号:          | Fixed Frame: | Topic:        |
| ------------------ | ------------ | ------------- |
| LDROBOT LiDAR LD14 | base_laser  | scan |


# Instructions

> This SDK is only applicable to the LiDAR products sold by Shenzhen LDROBOT Co., LTD. The product models are :
> - LDROBOT LiDAR LD14

## 0. get LiDAR ROS2 Package
```bash
$ cd ~

$ mkdir -p ldlidar_ros2_ws/src

$ cd ldlidar_ros2_ws/src

$ git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
# or
$ git clone  https://gitee.com/ldrobotSensorTeam/ldlidar_sl_ros2.git
```
## step 1: system setup
- Connect the LiDAR to your system motherboard via an onboard serial port or usB-to-serial module (for example, CP2102 module).

- Set the -x permission for the serial port device mounted by the radar in the system (for example, /dev/ttyUSB0)

  - In actual use, the LiDAR can be set according to the actual mounted status of your system, you can use 'ls -l /dev' command to view.

``` bash
$ cd ~/ldlidar_ros2_ws

$ sudo chmod 777 /dev/ttyUSB0
```
- Modify the `port_name` value in the Lanuch file corresponding to the radar product model under `launch/`, using `ld14.launch.py` as an example, as shown below.

```py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'topic_name': 'scan'},
        {'port_name': '/dev/ttyUSB0'},
        {'frame_id': 'base_laser'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
---
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  return LaunchDescription([
    Node(
      # LDROBOT LiDAR publisher node
      package='ldlidar_sl_ros2',
      executable='ldlidar_sl_ros2_node',
      name='ldlidar_publisher_ld14',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14'},
        {'topic_name': 'scan'},
        {'port_name': '/dev/ttyUSB0'},
        {'frame_id': 'base_laser'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 135.0},
        {'angle_crop_max': 225.0}
      ]
    )
  ])
```

## step 2: build

Run the following command.

```bash
$ cd ~/ldlidar_ros2_ws

$ colcon build
```
## step 3: run

```bash
$ source install/setup.bash
```
- The product is LDROBOT LiDAR LD14

  ``` bash
  $ ros2 launch ldlidar_sl_ros2 ld14.launch.py
  ```


## step 3: test

> The code supports ubuntu 20.04 ros2 foxy version and above, using rviz2 visualization.

- new a terminal (Ctrl + Alt + T) and use Rviz2 tool(run command: `rviz2`) ,open the `ldlidar.rviz` file below the rviz2 folder of the readme file directory
```bash
$ rviz2
```

| Product:          | Fixed Frame: | Topic:        |
| ------------------ | ------------ | ------------- |
| LDROBOT LiDAR LD14 | base_laser  | scan   |
