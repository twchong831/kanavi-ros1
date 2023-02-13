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
  - [Directories](#directories)
    - [CARNAVICOM\_LIB](#carnavicom_lib)
    - [carnavicomlidar\_ros](#carnavicomlidar_ros)
  - [Update Log](#update-log)
    - [2022-06-29](#2022-06-29)
    - [2022-07-04](#2022-07-04)
    - [2022-07-05](#2022-07-05)
    - [2022-09-07](#2022-09-07)
    - [2022-09-21](#2022-09-21)
    - [2023-01-25](#2023-01-25)
    - [2023-01-26](#2023-01-26)
    - [2023-02-13](#2023-02-13)

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
| ✅   | **VL-R004IK02** | 4ch 90&deg; LiDAR Sensor (R4)    |

## How to Build Kanavi-mobility LiDAR ROS Package

```sh
cd ros
make all    # install carnavicom LiDAR LIB and ROS driver
# if) make lib : install carnavicom LiDAR LIB only
# if) make ros : install carnavicom LiDAR ROS driver only
```

## How to start ROS Node

```sh
rosrun carnavicomlidar_ros carnavicomlidar_ros -h    # output node active option

# if VL-AS16
rosrun carnavicomlidar_ros carnavicomlidar_ros -i 192.168.123.99 5000

# if VL-R002IF01 / VL-R0001IK02
# using UDP multicast
rosrun carnavicomlidar_ros carnavicomlidar_ros -i 192.168.123.99 5000 -m 224.0.0.5

# make config(.ini) file
# save the configuration such as IP, port num
rosrun carnavicomlidar_ros carnavicomlidar_ros -i 192.168.123.99 5000 -fs config.ini

# read config(.ini) file
rosrun carnavicomlidar_ros carnavicomlidar_ros -fl ~/catkin_ws/src/carnavicomlidar_ros/config/config.ini
```

### RVIZ visualization Result

#### VL-R002IK01 (R2)

![output-R4](https://github.com/kanaviMobility/ROS/blob/b61708296beb582d901c588b770c1e037260ef03/ros/image/output_R2.png)

#### VL-R004IK02 (R4)

![output-R4](https://github.com/kanaviMobility/ROS/blob/83151f9fefc123cacc877ce8deb81573dee3af0d/ros/image/output_R4.png)

## Directories

### CARNAVICOM_LIB

- Library for Carnavicom LiDAR sensor processing
- to connect LiDAR sensor using UDP
- to process LiDAR raw Data to Datagram(include/header.h/carnaviDatagram)

```sh
/usr/local/lib                            # Library path
/usr/local/include/CARNAVICOM_LIB         # include path
```

### carnavicomlidar_ros

- ROS node for Carnavicom LiDAR sensor
- to convert LiDAR raw data to PointXYZRGB(by pcl)

```sh
~/catkin_ws/src/carnavicomlidar_ros    # ros node src Path
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

- update Company name(carnavicom->kanavi-mobility)

### 2023-02-13

- update & add VL-R004IK02 FUNC.
