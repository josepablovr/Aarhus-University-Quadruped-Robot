#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/buttons.hpp"
#include "custom_interfaces/msg/leg_readings.hpp"
#include "custom_interfaces/msg/leg_commands.hpp"
#include "custom_interfaces/msg/leds.hpp"

#include "comm_interface/Leg_CAN_Network.h"

class ButtonProcessorNode : public rclcpp::Node {
private:
    rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr leg_readings_publisher_;
    rclcpp::Subscription<custom_interfaces::msg::Buttons>::SharedPtr subscription_;
    rclcpp::Subscription<custom_interfaces::msg::LegCommands>::SharedPtr leg_commands_subscription_;
    rclcpp::Publisher<custom_interfaces::msg::Leds>::SharedPtr led_publisher_;
    
    // Variables to store the current state
    int state_ = 0;
    int control_mode_ = 2;
    float position_cmds_[3];
    int position_cmds_speeds_[3];
    float torque_cmds_[3];
public:
    ButtonProcessorNode() : Node("button_processor_node"), state_(IDLE) {
        // Publisher for "leg_readings" topic
        leg_readings_publisher_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings", 10);

        // Publisher for "led_state" topic
        led_publisher_ = this->create_publisher<custom_interfaces::msg::Leds>("led_state", 10);

        // Subscriber for "buttons_state" topic
        subscription_ = this->create_subscription<custom_interfaces::msg::Buttons>(
            "buttons_state", 10, std::bind(&ButtonProcessorNode::buttons_state_callback, this, std::placeholders::_1));
        
        leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>(
            "leg_commands", 10, std::bind(&ButtonProcessorNode::leg_commands_callback, this, std::placeholders::_1));
            
        const char* interface = "can1"; // Example interface name
        
        
        // Run the switch-case logic continuously in a separate thread
        std::thread([&, this]() {
            Leg_CAN_Network legCAN(interface);
            while (rclcpp::ok()) {
                legCAN.runCommunication(state_, control_mode_, position_cmds_, position_cmds_speeds_, torque_cmds_);
                // Sleep for a short duration to avoid tight loop
                //std::cerr << legCAN.corrected_encoders[2]<< std::endl;
                
                custom_interfaces::msg::LegReadings leg_readings_msg;
                leg_readings_msg.shoulder_joint_angle = legCAN.corrected_encoders[0];
                leg_readings_msg.hip_joint_angle = legCAN.corrected_encoders[1];
                leg_readings_msg.knee_joint_angle = legCAN.corrected_encoders[2];
                // Speeds
                
                leg_readings_msg.shoulder_joint_speed = legCAN.corrected_speeds[0];
                leg_readings_msg.hip_joint_speed = legCAN.corrected_speeds[1];
                leg_readings_msg.knee_joint_speed = legCAN.corrected_speeds[2];
                
                
                //Torques
                leg_readings_msg.shoulder_joint_torque = legCAN.corrected_torques[0];
                leg_readings_msg.hip_joint_torque = legCAN.corrected_torques[1];
                leg_readings_msg.knee_joint_torque = legCAN.corrected_torques[2];
                
                
                leg_readings_publisher_->publish(leg_readings_msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }).detach();

    }

private:
    void buttons_state_callback(const custom_interfaces::msg::Buttons::SharedPtr msg) {
        // Update the state variables based on the callback
        // This is just an example, replace with your actual logic
        state_ = static_cast<MainState>(msg->state);
    }
    
    void leg_commands_callback(const custom_interfaces::msg::LegCommands::SharedPtr msg) {
        // Update the state variables based on the callback
        // This is just an example, replace with your actual logic
        ;
    }
   
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
