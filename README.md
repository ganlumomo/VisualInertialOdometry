# VisualInertialOdometry

## Introduction

This project is designed for students to learn the front-end and back-end in a Simultaneous Localization and Mapping (SLAM) system. The objective is that using feature_tracker in [VINS-MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) as front-end, and [GTSAM](https://github.com/borglab/gtsam) as back-end to implement a visual inertial odometry (VIO) algorithm for real-data collected by a vehicle: [The MVSEC Dataset](https://daniilidis-group.github.io/mvsec/). The code is modified based on original code from [CPI](https://github.com/rpng/cpi).

Specifically, we are learning how to utilize a front-end package and use IMUFactor, SmartProjectionPoseFactor and ISAM2 optimizer in GTSAM to achieve a simple but straight-forward VIO system.

## Instruction

### Step 1

Install ROS: 

Ubuntu 18.04: [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu).

Ubuntu 16.04: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu).

Install GTSAM as a thirdparty: [https://github.com/borglab/gtsam](https://github.com/borglab/gtsam).

### Step 2

Clone this ros workspace:

```bash
$ git clone https://github.com/ganlumomo/VisualInertialOdometry.git
```

Try out the feature_tracker package to see how does it work and what does it output.

### Step 3
Refer examples [ImuFactorsExample.cpp](https://bitbucket.org/gtborg/gtsam/src/develop/examples/ImuFactorsExample.cpp) and [ISAM2Example_SmartFactor.cpp](https://bitbucket.org/gtborg/gtsam/src/develop/examples/ISAM2Example_SmartFactor.cpp) to implement the following functions defined in [GraphSolver.h](https://github.com/ganlumomo/VisualInertialOdometry/blob/master/src/gtsam_backend/include/gtsam_backend/GraphSolver.h#L115):
```bash
// For IMU preintegration
set_imu_preintegration
create_imu_factor
get_predicted_state
reset_imu_integration

// For smart vision factor
process_feat_smart
```
You are free to change the function signature, but make sure change other places accordingly.

### Step 4

Build the code and launch the ros node with rosbag data:
```bash
$ cd VisualInertialOdometry
$ catkin_make
$ source devel/setup.bash
$ roslaunch launch/mvsec_test.launch
$ rosbag play mvsec_test.bag
```
Test data can be downloaded here: [mvsec_test.bag](https://drive.google.com/file/d/1kjzadWbivMe3tH3uULDono-XcWwR_2l0/view?usp=sharing).

### Expected Results

<img src="https://raw.githubusercontent.com/ganlumomo/VisualInertialOdometry/master/github/expected_result.png">

Write one-page project summary about your implementation and result.

## References

VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator ([PDF](https://ieeexplore.ieee.org/document/8421746))

IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation ([PDF](http://www.roboticsproceedings.org/rss11/p06.pdf))

Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors ([PDF](https://ieeexplore.ieee.org/abstract/document/6907483))

The Multivehicle Stereo Event Camera Dataset: An Event Camera Dataset for 3D Perception ([PDF](https://ieeexplore.ieee.org/document/8288670))
