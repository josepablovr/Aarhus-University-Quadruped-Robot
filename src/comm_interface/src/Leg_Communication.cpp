#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "custom_interfaces/msg/buttons.hpp"
#include "custom_interfaces/msg/leg_readings.hpp"
#include "custom_interfaces/msg/leg_commands.hpp"
#include "custom_interfaces/msg/leds.hpp"
#include "comm_interface/CAN_BUS.h"
#include "comm_interface/RMD_X8.h"
#include "comm_interface/Leg_CAN_Network.h"

using custom_interfaces::msg::Buttons;

class ButtonProcessorNode : public rclcpp::Node {
private:
    rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr leg_readings_publisher_FL_leg_;
    rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr leg_readings_publisher_FR_leg_;
    rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr leg_readings_publisher_BL_leg_;
    rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr leg_readings_publisher_BR_leg_;
    
    
    rclcpp::Subscription<custom_interfaces::msg::Buttons>::SharedPtr control_interface_subscription_;
    rclcpp::Subscription<custom_interfaces::msg::LegCommands>::SharedPtr leg_commands_subscription_;
    rclcpp::Publisher<custom_interfaces::msg::Leds>::SharedPtr led_publisher_;
    
    
    
    // Variables to store the current state
    int state_ = 0;
    int control_mode_ = 2;
    float position_cmds_[3];
    int position_cmds_speeds_[3];
    float torque_cmds_[3];
    const char* interface_FL = "can1"; // Example interface name
    const char* interface_FR = "can0"; // Example interface name
    const char* interface_BL = "can3"; // Example interface name
    const char* interface_BR = "can2"; // Example interface name
    
    Leg FL_leg_memory;
    Leg FR_leg_memory;
    Leg BL_leg_memory;
    Leg BR_leg_memory;
    
    
public:
     ButtonProcessorNode() : Node("button_processor_node"), state_(IDLE)              {
        // Publisher for "leg_readings" topic
        
        unsigned char data_frame[8];   
        float knee_current_torque;
        //command_read_motor_status_2(data_frame); 
        knee_current_torque = get_motor_current(-20.0f);
    
        std::cout << knee_current_torque << std::endl;
        command_torque_control(data_frame, knee_current_torque);         
         // Print the contents of the data_frame array
        // Print the contents of the data_frame array
        std::cout << "Contents of the data_frame array:" << std::endl;
        std::cout << "0x";
        for (int i = 0; i < 8; ++i) {
            std::cout << " " << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(data_frame[i]);
        }
        std::cout << std::endl;
        
        Leg_CAN_Network legCAN_FL_(interface_FL);
        Leg_CAN_Network legCAN_FR_(interface_FR);
        Leg_CAN_Network legCAN_BL_(interface_BL);
        Leg_CAN_Network legCAN_BR_(interface_BR);
        
        FL_leg_memory = legCAN_FL_.leg;
        FR_leg_memory = legCAN_FR_.leg;
        BL_leg_memory = legCAN_BL_.leg;
        BR_leg_memory = legCAN_BR_.leg;
        
        position_cmds_[0] = 0.0f;
        position_cmds_[1] = 60.75f;
        position_cmds_[2] = -121.50f;

        // Initialize position_cmds_speeds_
        position_cmds_speeds_[0] = 1;
        position_cmds_speeds_[1] = 1;
        position_cmds_speeds_[2] = 1;
    
        // Initialize torque_cmds_
        torque_cmds_[0] = 0.0f;
        torque_cmds_[1] = 0.0f;
        torque_cmds_[2] = 0.0f;
        
       
    
    
        leg_readings_publisher_FL_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_FL", 10);
        leg_readings_publisher_FR_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_FR", 10);
        leg_readings_publisher_BL_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_BL", 10);
        leg_readings_publisher_BR_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_BR", 10);
        
        // Publisher for "led_state" topic
        led_publisher_ = this->create_publisher<custom_interfaces::msg::Leds>("led_state", 10);

        // Subscriber for "buttons_state" topic
        control_interface_subscription_ = this->create_subscription<custom_interfaces::msg::Buttons>(
            "buttons_state", 10, std::bind(&ButtonProcessorNode::buttons_state_callback, this, std::placeholders::_1));
        
        leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>(
            "leg_commands", 10, std::bind(&ButtonProcessorNode::leg_commands_callback, this, std::placeholders::_1));
            
        // Run the switch-case logic continuously in separate threads
        std::cout << "NETWORKS STARTED" << std::endl;
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_FL_), std::ref(leg_readings_publisher_FL_leg_), std::ref(FL_leg_memory), std::ref(position_cmds_) , std::ref(position_cmds_speeds_) , std::ref(torque_cmds_)).detach();
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_FR_), std::ref(leg_readings_publisher_FR_leg_), std::ref(FR_leg_memory), std::ref(position_cmds_) , std::ref(position_cmds_speeds_) , std::ref(torque_cmds_)).detach();
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_BL_), std::ref(leg_readings_publisher_BL_leg_), std::ref(BL_leg_memory), std::ref(position_cmds_) , std::ref(position_cmds_speeds_) , std::ref(torque_cmds_)).detach();
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_BR_), std::ref(leg_readings_publisher_BR_leg_), std::ref(BR_leg_memory), std::ref(position_cmds_) , std::ref(position_cmds_speeds_) , std::ref(torque_cmds_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(FL_leg_memory), std::ref(leg_readings_publisher_FL_leg_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(FR_leg_memory), std::ref(leg_readings_publisher_FR_leg_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(BL_leg_memory), std::ref(leg_readings_publisher_BL_leg_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(BR_leg_memory), std::ref(leg_readings_publisher_BR_leg_)).detach();
    }
private:

    void process(Leg_CAN_Network legCAN, rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr publisher, Leg& leg_memory, float position_cmds_[3], int position_cmds_speeds_[3], float torque_cmds_[3]) {
        
        auto last_pub_time = this->now();
        float joint_commands[3];
        joint_commands[0] = position_cmds_[0];
        joint_commands[1] = position_cmds_[1];
        joint_commands[2] = position_cmds_[2];
    
        int speed_commands[3];
        speed_commands[0] = position_cmds_speeds_[0];
        speed_commands[1] = position_cmds_speeds_[1];
        speed_commands[2] = position_cmds_speeds_[2];
    
        float torque_commands[3];
        torque_commands[0] = torque_cmds_[0];
        torque_commands[1] = torque_cmds_[1];
        torque_commands[2] = torque_cmds_[2];
        
        
        
        while (rclcpp::ok()) {
            
            //std::cout << "Should 1: " << legCAN.leg.shoulder.single_angle_position << std::endl;     
            legCAN.runCommunication(state_, control_mode_, joint_commands, speed_commands, torque_commands);
            leg_memory = legCAN.leg;
            //if ((this->now() - last_pub_time).seconds() >= 1.0 / 100)
            //{
                //custom_interfaces::msg::LegReadings leg_readings_msg;
                //leg_readings_msg.shoulder_joint_angle = legCAN.corrected_encoders[0];
                //leg_readings_msg.hip_joint_angle = legCAN.corrected_encoders[1];
                //leg_readings_msg.knee_joint_angle = legCAN.corrected_encoders[2];
                
                //leg_readings_msg.shoulder_joint_speed = legCAN.corrected_speeds[0];
                //leg_readings_msg.hip_joint_speed = legCAN.corrected_speeds[1];
                //leg_readings_msg.knee_joint_speed = legCAN.corrected_speeds[2];
                
                //leg_readings_msg.shoulder_joint_torque = legCAN.corrected_torques[0];
                //leg_readings_msg.hip_joint_torque = legCAN.corrected_torques[1];
                //leg_readings_msg.knee_joint_torque = legCAN.corrected_torques[2];
                
                //publisher->publish(leg_readings_msg);
                
                //last_pub_time = this->now();
            //}
        }
    }
    
    
     void publish_leg_readings() {
        rclcpp::Rate rate(100);  // 100 Hz
        
        while (rclcpp::ok()) {
            publish_leg_reading(FL_leg_memory, leg_readings_publisher_FL_leg_);
            publish_leg_reading(FR_leg_memory, leg_readings_publisher_FR_leg_);
            publish_leg_reading(BL_leg_memory, leg_readings_publisher_BL_leg_);
            publish_leg_reading(BR_leg_memory, leg_readings_publisher_BR_leg_);
            
            rate.sleep();  // Maintain the desired publishing rate
        }
    }
    
    void publish_leg_reading(Leg& leg_memory, rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr publisher) {
        
        rclcpp::Rate rate(100);
        while (rclcpp::ok()) {
            custom_interfaces::msg::LegReadings leg_readings_msg;
            leg_readings_msg.shoulder_joint_angle = leg_memory.shoulder.joint_angle;
            leg_readings_msg.hip_joint_angle = leg_memory.hip.joint_angle;
            leg_readings_msg.knee_joint_angle = leg_memory.knee.joint_angle;
            
            leg_readings_msg.shoulder_joint_speed = leg_memory.shoulder.joint_speed;
            leg_readings_msg.hip_joint_speed = leg_memory.hip.joint_speed;
            leg_readings_msg.knee_joint_speed = leg_memory.knee.joint_speed;
            
            leg_readings_msg.shoulder_joint_torque = leg_memory.shoulder.joint_torque;
            leg_readings_msg.hip_joint_torque = leg_memory.hip.joint_torque;
            leg_readings_msg.knee_joint_torque = leg_memory.knee.joint_torque;
            
            leg_readings_msg.failed_messages = leg_memory.failed_messages;
             
            publisher->publish(leg_readings_msg);
            rate.sleep();
        }
    }
        
    void buttons_state_callback(const custom_interfaces::msg::Buttons::SharedPtr msg) {
        state_ = static_cast<MainState>(msg->state);
        //std::cerr << "Button Received" << state_ <<std::endl;
    }
    
    void leg_commands_callback(const custom_interfaces::msg::LegCommands::SharedPtr msg) {
        // Update the state variables based on the callback
    }
   
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
