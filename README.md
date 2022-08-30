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
