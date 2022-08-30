# livox_projection

## What does this code do?

1. Reads the camera intrinsic data and correct camera distortion
2. Reads the extrinsic calibration data and project the pointcloud onto the image frame
3. Subscribe to `/camera/image_0`
4. Subscribe to `BoundingBox2Darray` msg from the yolov5 model
5. Query and publish the average distance to the points in the bounding box to topic `/distance`

## Download source code

First, download the source code to the user root directory.
If you want to organize it in a single ROS workspace, then you will need to modify the commands.

```bash
cd ~
git clone https://github.com/ece191-team-b/livox_projection
cd ~/livox_projection
```

## Install dependencies

The projection code depends on multiple ROS packages, but most of them should be already installed if you can launch the cameras and lidars.
The only one that is specific to this package is `ros-galactic-pcl-ros`.

Install it using the following command:

```bash
sudo apt install ros-galactic-pcl-ros
```

## Build the package

In order for the parameters stored in the launch file `param.launch.py` to be able to be changed dynamically without rebuilding the packages, we need to colcon build with the argument `--symlink-install`. This will create a symbolic link in the `~/livox_projection/install/` folder of the launch file instead of making a copy.

To change the launch parameters, simply modify `param.launch.py`.

```bash
cd ~/livox_projection
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source ./install/setup.bash
```

## Navigating to the repo folder and launching ros node

```bash
ros2 launch livox_pcd_projection param.launch.py
```

## Calibration results

Before you use the code, make sure the intrinsic and extrisic calibration results are included in the root directory of the package:

```bash
~/livox_projection/calibration_data/parameters/intrinsic.txt
~/livox_projection/calibration_data/parameters/extrinsic.txt
```

These files should be automatically generated from the `camera_lidar_calibration` (provide link here).

A default calibration results specific to the RobotX's camera lidar mount is already included in this repository, but in the case that a more up-to-date calibration results is generated, you can simply copy those to the folder without making any changes to those files.

## Making changes to `param.launch.py`

There are a few things you can change in `param.launch.py` to change the behavior of the code.

1. File path of the intrinsic calibration result
2. File path of the extrinsic calibration result
3. Name of the Livox lidar custom pointcloud topic to subscribe to
4. Name of the Image topic to subscribe to
5. Decay rate of the projected lidar pointcloud
6. Refresh rate of the ROS node (lidar max fps is 10, so exceeding that are unlikely to improve performance)
7. Debug mode
