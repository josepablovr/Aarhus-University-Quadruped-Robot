# AU Quadruped Robot ROS2 Code

## Description
ROS2 drivers to control the RMD-X8 DC Brushless Motors with position and torque commands.

IMU Drivers for BNO055 based on [Evan Flynn work](https://github.com/flynneva/bno055)



#### Build
   
Perform a build of your workspace
    
    cd ~/ros2_ws
    colcon build




### Launch Control Interface Node

    ros2 run control_interface_package button_reader

### Launch IMU Node
Run with customized parameter file:

    ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml
    


### Launch Actuators Node

    ros2 run comm_interface leg_comm


### Launch Crawl Gait Node

    ros2 run gait_pkg crawl_gait
    
    
### Launch State Estimator

    ros2 run estimation_pkg state_estimation
    
    
### Software Arquitecture Implementation

![Control Arquitecture](Robot_Software_Arquitecture.jpg)


### Control Interface Basic Test
https://youtube.com/shorts/MwS2y61EkHA?feature=shared

