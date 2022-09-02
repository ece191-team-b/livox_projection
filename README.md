# 1. ECE 191 Team B - Perception Livox Projection ROS Node

- [1. ECE 191 Team B - Perception Livox Projection ROS Node](#1-ece-191-team-b---perception-livox-projection-ros-node)
  - [1.1. What does this code do?](#11-what-does-this-code-do)
    - [1.1.1. Custom distance data specification](#111-custom-distance-data-specification)
  - [1.2. Installation and Usage](#12-installation-and-usage)
    - [1.2.1. Download source code](#121-download-source-code)
    - [1.2.2. Install dependencies](#122-install-dependencies)
    - [1.2.3. Build the package](#123-build-the-package)
    - [1.2.4. Calibration results](#124-calibration-results)
    - [1.2.5. Navigating to the repo folder and launching ros node](#125-navigating-to-the-repo-folder-and-launching-ros-node)
  - [1.3. Making changes to launch parameters](#13-making-changes-to-launch-parameters)

## 1.1. What does this code do?

1. Reads the camera intrinsic data and correct camera distortion
2. Reads the extrinsic calibration data and project the pointcloud onto the image frame
3. Subscribe to `BoundingBox2Darray` msg from the yolov5 model
4. Query and publish the average distance to the points in the bounding box to topic `/distance`

### 1.1.1. Custom distance data specification

The published `/distance` message uses custom message type `Dict` as defined [here](src/dist_msg/msg/Dist.msg).

It contains a `std_msgs/Header` data for time stamping. This allows nodes that subscribe to this topic to synchornize `/distance` messages with other topics using `message_filter` or other alternative methods.

One important thing to note is that the contents of `class_objs` and `distances` are correlated by their index. For instance, the first entry of `class_objs` has its distance data stored in the first entry of `distances`.

```bash
std_msgs/Header header      # ROS standard message header

uint8 count                 # total number of distance measurements
string[] obj_classes
float64[] distances 
```

## 1.2. Installation and Usage

The following section details the setup and usage of this ROS package.

### 1.2.1. Download source code

First, download the source code to the user root directory.
If you want to organize it in a single ROS workspace *(not recommended)*, then you will need to modify the commands.

```bash
cd ~
git clone https://github.com/ece191-team-b/livox_projection
cd ~/livox_projection
```

### 1.2.2. Install dependencies

The projection code depends on multiple ROS packages, but most of them should be already installed if you can launch the cameras and lidars. The only additional package is `ros-galactic-pcl-ros`.

Install it using the following command:

```bash
sudo apt install ros-galactic-pcl-ros
```

### 1.2.3. Build the package

In order for the parameters stored in the launch file `param.launch.py` to be able to be changed dynamically without rebuilding the packages, we need to colcon build with the argument `--symlink-install`. This will create a symbolic link in the `~/livox_projection/install/` folder of the launch file instead of making a copy.

**As of 09/02/2022 `--symlink-install` does not seem to be working for python config files. See [this github issue](https://github.com/colcon/colcon-core/issues/407)**

Therefore, currently, we need to rebuild the package every time we make changes to the launch file in the livox_pcd_projection package.

To change the launch parameters for projector, see this section [1.3. Making changes to launch parameters](#13-making-changes-to-launch-parameters).

```bash
cd ~/livox_projection
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source ./install/setup.bash
```

### 1.2.4. Calibration results

Before you use the code, make sure the intrinsic and extrisic calibration results are included in the root directory of the package:

```bash
~/livox_projection/calibration_data/parameters/intrinsic.txt
~/livox_projection/calibration_data/parameters/extrinsic.txt
```

Alternatively, you can also change the default path to somewhere else in the launch file located at [`src\livox_pcd_projection\config\projector.yaml`](src\livox_pcd_projection\config\projector.yaml).

These files should be automatically generated from the [`camera_lidar_calibration` node](https://github.com/ece191-team-b/livox_camera_lidar_calibration).


A default calibration results specific to the RobotX's camera lidar mount is already included in this repository, but in the case that a more up-to-date calibration results is generated, you can simply copy those to the folder without making any changes to those files.

### 1.2.5. Navigating to the repo folder and launching ros node

Make sure you have `livox_ros2_driver` launched. If not, refer to the instructions on [this page](https://github.com/ece191-team-b/livox_ros2_driver).

In an unused terminal (you can use the one that you used to run the commands above): launch the stamper, so that the lidar pointcloud messages can be synconized with the camera image messages.

```bash
cd ~/livox_projection
ros2 launch livox_pcd_projection stamper.launch.py
```

Open a new terminal and launch the projector node:

```bash
cd ~/livox_projection
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
ros2 run livox_pcd_projection projector.launch.py
```

## 1.3. Making changes to launch parameters

There are two config files `projector.yaml` and `stamper.yaml` provide to facilitate the launch of the projector and stamper nodes.

The `projector.yaml` file is located at [`src\livox_pcd_projection\config\projector.yaml`](src\livox_pcd_projection\config\projector.yaml).

and the `stamper.yaml` file is located at [`src\livox_pcd_projection\config\stamper.yaml`](src\livox_pcd_projection\config\stamper.yaml).

The `projector.yaml` file contains the following parameters:

1. Topic name of the camera Image to subscribe to
2. Topic name of the detected bounding boxes to subscribe to
3. Topic name the pointcloud to subscribe to (usually the output of the `stamper` node)
4. File path of the intrinsic calibration result
5. File path of the extrinsic calibration result
6. Name of the Livox lidar custom pointcloud topic to subscribe to
7. Decay rate of the projected lidar pointcloud
8. Refresh rate of the projector node
9. Debug mode switch
