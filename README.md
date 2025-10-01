# ORB_SLAM3_ROS2
This repository is the modified package of the [forked repository](https://github.com/jnskkmhr/orbslam3) and [original repository](https://github.com/zang09/ORB_SLAM3_ROS2). \
The issues I had are

* The node dies unexpectably.
* The pose sometimes jumps.
* The Initial X_coordiate of SLAM set to upward. This makes the pose to be estimated to go upward when the GT pose moves front.

Additionally, I added some extra topics for stereo-inertial mode. I refered this [ros1 wrapper](https://github.com/thien94/orb_slam3_ros).\

Standalone C++ ORB-SLAM3 repository is [here](https://github.com/yckim4042/ORB-SLAM3-STEREO-FIXED.git). \
It is modified for this repository.

---
---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites
- I have tested on below version.
  - Ubuntu 22.04
  - ROS2 Humble
  - OpenCV 4.5.4

- Build ORB_SLAM3
  - Go to this [repo](https://github.com/yckim4042/ORB-SLAM3-STEREO-FIXED) and follow build instruction.

- Install related ROS2 package
```
$ sudo apt install ros-$ROS_DISTRO-vision-opencv && sudo apt install ros-$ROS_DISTRO-message-filters
```

## How to build
1. Clone repository to your ROS workspace
```
$ mkdir -p colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone https://github.com/yckim4042/ORB_SLAM3_ROS2.git
```

2. Change this [line](https://github.com/yckim4042/orb_slam3_ros2/blob/main/CMakeLists.txt#L5) to your own `python site-packages` path

3. Change this [line](https://github.com/yckim4042/orb_slam3_ros2/blob/main/CMakeModules/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path

4. Unzip ORBvoc.txt.tar.gz file in /vocabulary folder

```
tar -xf ORBvoc.txt.tar.gz
```

Now, you are ready to build!
```
$ cd ~/colcon_ws
$ colcon build --symlink-install --packages-select orbslam3
```

## Troubleshootings
1. If you cannot find `sophus/se3.hpp`:  
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.
```
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```
2. Please compile with `OpenCV 4.5.4` version.

## How to use
1. Source the workspace  
```
$ source ~/colcon_ws/install/local_setup.bash
```

2. Run orbslam mode, which you want.  

I modified only stereo-inertial node from the original repo.


  - `MONO` mode  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  -  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` mode  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - ``STEREO & STEREO-INERTIAL` mode  
```
$ ros2 run orbslam3 PATH_TO_LAUNCH_FLIE
```
