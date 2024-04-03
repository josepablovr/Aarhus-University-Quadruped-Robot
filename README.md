# AU Quadruped Robot ROS2 Code

## Description
ROS2 drivers to control the Motors with position and torque commands

IMU Drivers for BNO055 based on [Evan Flynn work](https://github.com/flynneva/bno055)



#### Build
   
Perform a build of your workspace
    
    cd ~/ros2_ws
    colcon build


### Launch Control Interface Node

    ros2 run control_interface_package button_reader

### Launch Control Interface Node

    ros2 run control_interface_package button_reader

### Launch IMU Node
Run with customized parameter file:

    ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml
    


### Launch Actuators Node

    ros2 run comm_interface leg_comm




### Control Interface Basic Test

