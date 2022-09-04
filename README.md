# 1. ECE 191 Team B - Perception Livox Projection ROS Node

- [1. ECE 191 Team B - Perception Livox Projection ROS Node](#1-ece-191-team-b---perception-livox-projection-ros-node)
  - [1.1. What does this code do?](#11-what-does-this-code-do)
    - [1.1.1. Custom distance data specification](#111-custom-distance-data-specification)
  - [1.2. Installation and Usage](#12-installation-and-usage)
    - [1.2.1. Download source code](#121-download-source-code)
    - [1.2.2. Install dependencies](#122-install-dependencies)
    - [1.2.3. Build the package](#123-build-the-package)
    - [1.2.4. Calibration results](#124-calibration-results)
    - [1.2.5. Navigating to the repo folder and launching ROS node](#125-navigating-to-the-repo-folder-and-launching-ros-node)
  - [1.3. Making changes to launch parameters](#13-making-changes-to-launch-parameters)

## 1.1. What does this code do?

1. Reads the camera intrinsic data and correct camera distortion (debug mode only)
2. Reads the extrinsic calibration data and project the pointcloud onto the image frame
3. Subscribe to a `vision_msgs::msg::BoundingBox2Darray` topic published by the object detection model
4. Query and publish the median distance to the points in the bounding box to topic `/distances`

### 1.1.1. Custom distance data specification

The published `/distance` message uses custom message type `Dict` as defined [here](src/dist_msg/msg/Dist.msg).

It contains a `std_msgs/Header` data for time stamping. This allows nodes that subscribe to this topic to synchornize `/distances` messages with other topics using `message_filter` or other alternative methods.

One important thing to note is that the contents of `class_objs` and `distances` are correlated by their index. For instance, the first entry of `class_objs` has its distance data stored in the first entry of `distances`.

```c
std_msgs/Header header      # ROS standard message header

uint8 count                 # total number of distance measurements
string[] obj_classes
float64[] distances 
```

## 1.2. Installation and Usage

The following section details the setup and usage of this ROS package.

### 1.2.1. Download source code

First, download the source code to the user root directory.
If you want to organize it in a single ROS workspace *(not recommended)*, then you will need to modify the commands or move the folders around.

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

### 1.2.5. Navigating to the repo folder and launching ROS node

Make sure you have `livox_ros2_driver` launched (and `ros2 topic echo` to check if it is indeed publishing data). If not, refer to the instructions on [this page](https://github.com/ece191-team-b/livox_ros2_driver).

In an unused terminal (you can use the one that you used to run the commands above), run the following commands:

```bash
cd ~/livox_projection
ros2 launch livox_pcd_projection project_launch.launch.py
```

This launch file will launch both the `stamper` node and the `projector` node.

The stamper file is responsible for creating a time stamp for the pointcloud data. This is necessary because the pointcloud data published by the livox lidar does not have the correct UNIX time stamp. This is important because `message_filter` uses the time stamp data to synchronize the pointcloud data with the bounding box data.

There might be more elegant solution to this problem. See [livox's documentation on time synchronization](https://github.com/Livox-SDK/Livox-SDK/wiki/livox-device-time-synchronization-manual) for more information.

There are 4 different launch files you can use:

1. `project_launch.launch.py` - launches both the stamper and projector nodes
2. `debug_launch.launch.py` - launches the projector node with debug mode enabled
3. `projector.launch.py` - launches the projector node only (deprecated, not recommended to use)
4. `stamper.launch.py` - launches the stamper node only (deprecated, not recommended to use)

Usually, you will only need to use the first one. The other 3 are for debugging purposes.

`debug_launch.launch.py` will launch the projector node with debug mode enabled. This will cause the projector node to publish a image with the point cloud projection visualized to the topic `/debug_image`. This is useful if you want to validate if the calibration results are good or the distance measurements are correct.

However, **debug mode comes at a significant performance penalty**. Therefore, it is NOT recommend to use it when the system is deployed and running in real time.

## 1.3. Making changes to launch parameters

The config files are located in the `src\livox_pcd_projection\config` folder. The launch file is located in the `src\livox_pcd_projection\launch` folder.

Modify the yaml if you want to change the launch parameters. The launch file will automatically read the yaml file and use the parameters specified in it. If you want to change the launch behavior, you will need to modify the launch file itself.

Here is the content of the project.yaml file and a brief explanation of what each parameter does:

| Parameter name | Default value | Usage / Notes |
|---|---|---|
| camera_topic | /right_camera | camera image topic to subscribe to |
| detection_topic | /right_set/bbox | bounding box topic to subscribe to |
| lidar_topic | /livox/stamped | livox lidar custom pointcloud topic to subscribe to |
| intrinsic_path | ~/livox_projection/calibration_data/parameters/intrinsic.txt | where the camera intrinsic data is stored |
| extrinsic_path | ~/livox_projection/calibration_data/parameters/extrinsic.txt | where the camera-lidar extrinsic data is stored |
| lidar_threshold | 64000 | how many *latest* points of the lidar pointcloud will be kept and projected (decay rate) |
| refresh_rate |  30 | refresh rate of the projection node |
| debug | False | debug mode switch |
| bbox_size | 0.8 | how much to shrink the bounding boxes by (example: 1 = no shrinkage, 0.5 = width and height are halved while the center point is the same) |

| Parameter name | Default value | Usage / Notes |
|---|---|---|
|  input_topic | /livox/lidar | topic name of the livox custom lidar message |
| output_topic  | /livox/stamped | topic name of the to-be-published time-stamped message |
