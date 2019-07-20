# Deformable-Modeling
Reconstruct models of deformable objects using force probe technique and learning algorithm.

## System Overview
![arch](figures/architechture.png?raw=ture)

## Getting started
### Requirements

  * Hardware: UR5 (with force sensors), A computer with a dedicated GPU(recommended).
  * OS: Ubuntu 16.04
  * Necessary: UR API, Klamp't, Python 2.7

### Installation

### Calibration
you can do all when connecting the robot.
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
process can be physical or simulation.

### Version
- 2019.07.19 add line probe mode, add transform from probe to EE.
