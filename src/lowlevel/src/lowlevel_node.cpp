#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "custom_interfaces/msg/leg_readings.hpp"
#include "custom_interfaces/msg/leg_states.hpp"
#include "message/LowlevelState.h"
#include "common/unitreeRobot.h"
#include "control/Estimator.h"
#include "control/CtrlComponents.h"
#include <boost/bind/bind.hpp>  // Include the correct Boost header
#include <cmath>  // Include cmath for M_PI
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::placeholders;  // Use the std::placeholders namespace

class LowlevelNode : public rclcpp::Node {
public:
    LowlevelNode() : Node("lowlevel_node"), tf_broadcaster_(this), tf_broadcaster_FR(this), 
    tf_broadcaster_FL(this), tf_broadcaster_BR(this),tf_broadcaster_BL(this), ctrlComponents(ioInter) {
        //tf2_ros::TransformBroadcaster tf_broadcaster_(this);
        // Initialize subscribers
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_filtered", 10, std::bind(&LowlevelNode::imu_callback, this, _1));
        
        fl_subscriber_ = this->create_subscription<custom_interfaces::msg::LegReadings>(
            "/leg_readings_FL", 10, std::bind(&LowlevelNode::fl_leg_callback, this, _1));
        
        fr_subscriber_ = this->create_subscription<custom_interfaces::msg::LegReadings>(
            "/leg_readings_FR", 10, std::bind(&LowlevelNode::fr_leg_callback, this, _1));
        
        bl_subscriber_ = this->create_subscription<custom_interfaces::msg::LegReadings>(
            "/leg_readings_BL", 10, std::bind(&LowlevelNode::bl_leg_callback, this, _1));
        
        br_subscriber_ = this->create_subscription<custom_interfaces::msg::LegReadings>(
            "/leg_readings_BR", 10, std::bind(&LowlevelNode::br_leg_callback, this, _1));

        // Subscribe to leg_states topic
        leg_states_subscriber_ = this->create_subscription<custom_interfaces::msg::LegStates>(
            "/leg_states", 10, std::bind(&LowlevelNode::leg_states_callback, this, _1));


        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/estimation", 10);
        
        // Initialize CtrlComponents

         // Initialize CtrlComponents

        //IOInterface *ioInter;
        //ctrlComp = new CtrlComponents(ioInter);
        ctrlComponents.robotModel = new Go1Robot();  
        ctrlComponents.dt = 0.01; 
        ctrlComponents.geneObj();
        FL_received = false;
        FR_received = false;
        BL_received = false;
        BR_received = false;
        states_received = false;
        imu_received = false;
        (*ctrlComponents.contact)[0] = 1;
        (*ctrlComponents.contact)[1] = 1;
        (*ctrlComponents.contact)[2] = 1;
        (*ctrlComponents.contact)[3] = 1;


    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        ctrlComponents.lowState->imu.quaternion[0] = msg->orientation.w;
        ctrlComponents.lowState->imu.quaternion[1] = msg->orientation.x;
        ctrlComponents.lowState->imu.quaternion[2] = msg->orientation.y;
        ctrlComponents.lowState->imu.quaternion[3] = msg->orientation.z;

        ctrlComponents.lowState->imu.gyroscope[0] = msg->angular_velocity.x;
        ctrlComponents.lowState->imu.gyroscope[1] = msg->angular_velocity.y;
        ctrlComponents.lowState->imu.gyroscope[2] = msg->angular_velocity.z;

        ctrlComponents.lowState->imu.accelerometer[0] = msg->linear_acceleration.x;
        ctrlComponents.lowState->imu.accelerometer[1] = msg->linear_acceleration.y;
        ctrlComponents.lowState->imu.accelerometer[2] = msg->linear_acceleration.z;

        if (!imu_received){
        print_imu_state();
        imu_received = true;}
    }

    void fl_leg_callback(const custom_interfaces::msg::LegReadings::SharedPtr msg) {
        update_leg_state(msg, 1);
        if (!FL_received){
            print_leg_state(0);
            FL_received = true;}
    }

    void fr_leg_callback(const custom_interfaces::msg::LegReadings::SharedPtr msg) {
        update_leg_state(msg, 0);
        if (!FR_received){
            print_leg_state(1);
            FR_received = true;}
    }

    void bl_leg_callback(const custom_interfaces::msg::LegReadings::SharedPtr msg) {
        update_leg_state(msg, 3);
        if (!BL_received){
            print_leg_state(2);
            BL_received = true;}
            
            
        if (FL_received && FR_received && BL_received && BR_received && states_received && imu_received) {
    // All variables are true
            //RCLCPP_INFO(this->get_logger(), "READY");
            ctrlComponents.estimator->run();
            Vec3 position = ctrlComponents.estimator->getPosition();     
            //RCLCPP_INFO(this->get_logger(), "Position: [x: %f, y: %f, z: %f]", position[0], position[1], position[2]);            

            auto odom = nav_msgs::msg::Odometry();
           
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "world";
            odom.child_frame_id = "base_link";
 
            odom.pose.pose.position.x = position[0];
            odom.pose.pose.position.y = position[1];
            odom.pose.pose.position.z = position[2];            
            
            odom.pose.pose.orientation.x = ctrlComponents.lowState->imu.quaternion[0];
            odom.pose.pose.orientation.y = ctrlComponents.lowState->imu.quaternion[1];
            odom.pose.pose.orientation.z = ctrlComponents.lowState->imu.quaternion[2];
            odom.pose.pose.orientation.w = ctrlComponents.lowState->imu.quaternion[3];
            
            odometry_publisher_->publish(odom);
            
            }

        
    }

    void br_leg_callback(const custom_interfaces::msg::LegReadings::SharedPtr msg) {
        update_leg_state(msg, 2);
        if (!BR_received){
            print_leg_state(3);
            BR_received = true;}
        
    }

    void update_leg_state(const custom_interfaces::msg::LegReadings::SharedPtr msg, int leg_index) {
        int base_index = leg_index * 3;
        ctrlComponents.lowState->motorState[base_index].q = msg->shoulder_joint_angle*M_PI / 180.0;
        ctrlComponents.lowState->motorState[base_index].dq = msg->shoulder_joint_speed*M_PI / 180.0;
        ctrlComponents.lowState->motorState[base_index].tauEst = msg->shoulder_joint_torque;

        ctrlComponents.lowState->motorState[base_index + 1].q = msg->hip_joint_angle*M_PI / 180.0;
        ctrlComponents.lowState->motorState[base_index + 1].dq = msg->hip_joint_speed*M_PI / 180.0;
        ctrlComponents.lowState->motorState[base_index + 1].tauEst = msg->hip_joint_torque;

        ctrlComponents.lowState->motorState[base_index + 2].q = msg->knee_joint_angle*M_PI / 180.0;
        ctrlComponents.lowState->motorState[base_index + 2].dq = msg->knee_joint_speed*M_PI / 180.0;
        ctrlComponents.lowState->motorState[base_index + 2].tauEst = msg->knee_joint_torque;
    }

    void print_imu_state() {
        RCLCPP_INFO(this->get_logger(), "IMU State:");
        RCLCPP_INFO(this->get_logger(), "  Orientation: [w: %f, x: %f, y: %f, z: %f]",
                    ctrlComponents.lowState->imu.quaternion[0], ctrlComponents.lowState->imu.quaternion[1], 
                    ctrlComponents.lowState->imu.quaternion[2], ctrlComponents.lowState->imu.quaternion[3]);
        RCLCPP_INFO(this->get_logger(), "  Gyroscope: [x: %f, y: %f, z: %f]",
                    ctrlComponents.lowState->imu.gyroscope[0], ctrlComponents.lowState->imu.gyroscope[1], 
                    ctrlComponents.lowState->imu.gyroscope[2]);
        RCLCPP_INFO(this->get_logger(), "  Accelerometer: [x: %f, y: %f, z: %f]",
                    ctrlComponents.lowState->imu.accelerometer[0], ctrlComponents.lowState->imu.accelerometer[1], 
                    ctrlComponents.lowState->imu.accelerometer[2]);
    }

    void print_leg_state(int leg_index) {
        int base_index = leg_index * 3;
        const char* leg_names[] = {"FL", "FR", "BL", "BR"};
        const char* leg_name = leg_names[leg_index];
        
        RCLCPP_INFO(this->get_logger(), "%s Leg State:", leg_name);
        RCLCPP_INFO(this->get_logger(), "  Shoulder Joint: [Angle: %f, Speed: %f, Torque: %f]",
                    ctrlComponents.lowState->motorState[base_index].q, ctrlComponents.lowState->motorState[base_index].dq, 
                    ctrlComponents.lowState->motorState[base_index].tauEst);
        RCLCPP_INFO(this->get_logger(), "  Hip Joint: [Angle: %f, Speed: %f, Torque: %f]",
                    ctrlComponents.lowState->motorState[base_index + 1].q, ctrlComponents.lowState->motorState[base_index + 1].dq, 
                    ctrlComponents.lowState->motorState[base_index + 1].tauEst);
        RCLCPP_INFO(this->get_logger(), "  Knee Joint: [Angle: %f, Speed: %f, Torque: %f]",
                    ctrlComponents.lowState->motorState[base_index + 2].q, ctrlComponents.lowState->motorState[base_index + 2].dq, 
                    ctrlComponents.lowState->motorState[base_index + 2].tauEst);
    }

    void leg_states_callback(const custom_interfaces::msg::LegStates::SharedPtr msg) {
        // Update contact vector based on leg states


        for (int i = 0; i < 4; ++i) {
            (*ctrlComponents.contact)[i] = (msg->leg_states[i] == 0) ? 1 : 0;
        }
        (*ctrlComponents.contact)[0] = (msg->leg_states[1] == 0) ? 1 : 0;
        (*ctrlComponents.contact)[1] = (msg->leg_states[0] == 0) ? 1 : 0;
        (*ctrlComponents.contact)[2] = (msg->leg_states[3] == 0) ? 1 : 0;
        (*ctrlComponents.contact)[3] = (msg->leg_states[2] == 0) ? 1 : 0;
        //RCLCPP_INFO(this->get_logger(), "Updated contact vector: [%d, %d, %d, %d]",
                   // (*ctrlComponents.contact)[0], (*ctrlComponents.contact)[1], (*ctrlComponents.contact)[2], (*ctrlComponents.contact)[3]);

        if (!states_received){
            RCLCPP_INFO(this->get_logger(), "Updated contact vector: [%d, %d, %d, %d]",
                    (*ctrlComponents.contact)[0], (*ctrlComponents.contact)[1], (*ctrlComponents.contact)[2], (*ctrlComponents.contact)[3]);

            states_received = true;}
        // Print the updated contact vector
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::LegReadings>::SharedPtr fl_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::LegReadings>::SharedPtr fr_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::LegReadings>::SharedPtr bl_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::LegReadings>::SharedPtr br_subscriber_;
    rclcpp::Subscription<custom_interfaces::msg::LegStates>::SharedPtr leg_states_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::TransformBroadcaster tf_broadcaster_FR;
    tf2_ros::TransformBroadcaster tf_broadcaster_FL;
    tf2_ros::TransformBroadcaster tf_broadcaster_BR;
    tf2_ros::TransformBroadcaster tf_broadcaster_BL;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    // Define CtrlComponents
    CtrlComponents ctrlComponents;
    IOInterface *ioInter;
    bool FL_received;
    bool FR_received;
    bool BL_received;
    bool BR_received;
    bool states_received;
    bool imu_received;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowlevelNode>());
    rclcpp::shutdown();
    return 0;
}
