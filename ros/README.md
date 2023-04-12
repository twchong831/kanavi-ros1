# Kanavi-mobility co., ltd., LiDAR ROS Driver

## Contents

- [Kanavi-mobility co., ltd., LiDAR ROS Driver](#kanavi-mobility-co-ltd-lidar-ros-driver)
  - [Contents](#contents)
  - [Development ENV](#development-env)
  - [Support LiDAR Device](#support-lidar-device)
  - [How to Build Kanavi-mobility LiDAR ROS Package](#how-to-build-kanavi-mobility-lidar-ros-package)
  - [How to start ROS Node](#how-to-start-ros-node)
    - [RVIZ visualization Result](#rviz-visualization-result)
      - [VL-R002IK01 (R2)](#vl-r002ik01-r2)
      - [VL-R004IK02 (R4)](#vl-r004ik02-r4)
      - [AXES](#axes)
        - [1](#1)
        - [2](#2)
  - [Directories](#directories)
    - [KANAVI\_LIB](#kanavi_lib)
    - [kanavilidar\_ros](#kanavilidar_ros)
  - [Update Log](#update-log)
    - [2022-06-29](#2022-06-29)
    - [2022-07-04](#2022-07-04)
    - [2022-07-05](#2022-07-05)
    - [2022-09-07](#2022-09-07)
    - [2022-09-21](#2022-09-21)
    - [2023-01-25](#2023-01-25)
    - [2023-01-26](#2023-01-26)
    - [2023-02-13](#2023-02-13)
    - [2023-02-14](#2023-02-14)
    - [2023-03-28](#2023-03-28)

## Development ENV

| OS version  |       Ubuntu 20.04 |
| :---------- | -----------------: |
| ROS version | ros-noetic-desktop |

## Support LiDAR Device

|     | Name            | Description                      |
| --- | --------------- | -------------------------------- |
| ✅   | **VL-R016AK01** | 16ch 145&deg; LiDAR Sensor       |
| ⬜️   | VL-L001IK01     | 1ch laser range finder           |
| ⬜️   | VL-R001IK01 | 1ch 120&deg; LiDAR Sensor        |
| ✅   | **VL-R002IK01** | 2ch 120&deg; LiDAR Sensor (R2)   |
| ✅   | **VL-R001IK02**     | 1ch 300&deg; LiDAR Sensor (R300) |
| ⬜️   | VL-R004IK01     | 4ch 90&deg; LiDAR Sensor?        |
| ✅   | **VL-R004IK02** | 4ch 100&deg; LiDAR Sensor (R4)    |

## How to Build Kanavi-mobility LiDAR ROS Package

```sh
cd ros
make all    # install kanavi LiDAR LIB and ROS driver
# if) make lib : install kanavi LiDAR LIB only
# if) make ros : install kanavi LiDAR ROS driver only
```

## How to start ROS Node

```sh
rosrun kanavilidar_ros kanavilidar_ros -h    # output node active options

# if VL-AS16
rosrun kanavilidar_ros kanavilidar_ros -i 192.168.123.99 5000

# if VL-R002IF01 / VL-R0001IK02
# using UDP multicast
rosrun kanavilidar_ros kanavilidar_ros -i 192.168.123.99 5000 -m 224.0.0.5

# change base Axes
rosrun kanavilidar_ros kanavilidar_ros -i 192.168.123.99 5000 -m 224.0.0.5 -axes 1
rosrun kanavilidar_ros kanavilidar_ros -i 192.168.123.99 5000 -m 224.0.0.5 -axes 2

# make config(.ini) file
# save the configuration such as IP, port num
rosrun kanavilidar_ros kanavilidar_ros -i 192.168.123.99 5000 -fs config.ini

# read config(.ini) file
rosrun kanavilidar_ros kanavilidar_ros -fl ~/catkin_ws/src/kanavilidar_ros/config/config.ini
```

### RVIZ visualization Result

#### VL-R002IK01 (R2)

![output-R4](https://github.com/kanaviMobility/ROS/blob/b61708296beb582d901c588b770c1e037260ef03/ros/image/output_R2.png)

#### VL-R004IK02 (R4)

![output-R4](https://github.com/kanaviMobility/ROS/blob/83151f9fefc123cacc877ce8deb81573dee3af0d/ros/image/output_R4.png)

#### AXES

##### 1

- base Axes
![base axes](https://github.com/kanaviMobility/ROS/blob/15c4400f8d59ec23de53a0d3b7421837af1d274b/ros/image/axes1.png)

- result
![base-axes-vlas16](https://github.com/kanaviMobility/ROS/blob/15c4400f8d59ec23de53a0d3b7421837af1d274b/ros/image/base_axes.png)

##### 2

- base Axes
![base axes2](https://github.com/kanaviMobility/ROS/blob/15c4400f8d59ec23de53a0d3b7421837af1d274b/ros/image/axes2.png)

- result
![change-axes-vlas16](https://github.com/kanaviMobility/ROS/blob/15c4400f8d59ec23de53a0d3b7421837af1d274b/ros/image/change_axes_2.png)

## Directories

### KANAVI_LIB

- Library for Kanavi-Mobility LiDAR sensor processing
- to connect LiDAR sensor using UDP
- to process LiDAR raw Data to Datagram(include/header.h/lidarDatagram)

```sh
/usr/local/lib                            # Library path
/usr/local/include/KANAVI_LIB         # include path
```

### kanavilidar_ros

- ROS node for Kanavi-Mobility LiDAR sensor
- to convert LiDAR raw data to PointXYZRGB(by pcl)

```sh
~/catkin_ws/src/kanavilidar_ros    # ros node src Path
```

## Update Log

### 2022-06-29

- first Release.

### 2022-07-04

- rewrite Error & add comments of Func.

### 2022-07-05

- add LICENSE(BSD3)

### 2022-09-07

- Makefile & CMakeLists.txt lib name error rewrite.

### 2022-09-21

- R2 LiDAR sensor 0 ch data loss error Updated.

### 2023-01-25

- make git repo
- add support OS list in Lib

### 2023-01-26

- update Company name(canavi->kanavi-mobility)

### 2023-02-13

- update & add VL-R004IK02 FUNC.

### 2023-02-14

- update all Compayname(carnavicom to kanavi-mobility)

### 2023-03-28

- to update for Select Axes
