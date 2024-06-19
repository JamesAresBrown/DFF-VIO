# DFF-VIO

## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**

Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**

Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build DFF-VIO

Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/JamesAresBrown/DFF-VIO.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. RUN

```
roslaunch vins vins_rviz.launch
rosrun vins vins_node_wait [Configuration Files. e.g. ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml]
(optional) rosrun loop_fusion loop_fusion_node [Configuration Files. e.g. ~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_mono_imu_config.yaml]
rosbag play [rosbag file. e.g. YOUR_DATASET_FOLDER/MH_01_easy.bag]
```
