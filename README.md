# livox_projection

## Download source code

```bash
cd ~
git clone https://github.com/ece191-team-b/livox_projection
cd ~/livox_projection
```

## Install dependencies

```bash
sudo apt install ros-galactic-pcl-ros
```

## Build the package

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

## What does this code do?

1. Reads the camera intrinsic data and correct camera distortion
2. Reads the extrinsic calibration data and project the pointcloud onto the image frame
3. Subscribe to `/camera/image_0`
4. Subscribe to `BoundingBox2D`
5. Query and publish the average distance to the points in the bounding box to topic `/topic`
