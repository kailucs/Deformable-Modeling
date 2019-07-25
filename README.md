# Deformable-Modeling
Construct models of deformable objects using force-probe contacting and learning algorithm.

## System Overview
![arch](figures/architechture.png?raw=ture)

## Getting started
### Requirements

  * Hardware:

  * - UR5 (with force sensors)

    - A computer with a dedicated GPU(recommended)
    - Intel RealSense Camera SR300
    - Probe (can be any type)

  * OS:  > Ubuntu 16.04

  * Environment:

    - Python 2.7

    - UR API
    - Klamp't
    - librealsense & python API
    - open3d
    - numpy

### Installation

Simply git clone the project.

### Calibration
First, do camera calibration to make the system work correctly. You can do all when connecting the robot.
```
python calibration_testing.py
```
or
```
cd utils
python calibration_calculation.py
python generate_marker_transform_color.py
```
then connect robot
```
python take_calibration_pictures
```

### Executing
simply run
```
python main.py --process [process-name]
```
process can be physical or debugging.

**or (recommend)**

check`config.py`, set `self.use_mkdir_exp = True` , then 

```
python main.py --process debugging --step y --probe point 
```

### Version

- 2019.07.16 first version.
- 2019.07.19 add line probe mode, add transform from probe to EE, fix unit vector bug.
- 2019.07.21 add duplicates removing, collision detection, and fix point cloud order.
- 2019.07.23 change line poking process: first reconstruct a probe list then poke. fix some vis bugs.
- 2019.07.24 speed up getting pcd, test all process in physical, update data format, add future folder (2 camera in thread)
