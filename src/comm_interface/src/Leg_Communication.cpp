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
    rclcpp::Subscription<custom_interfaces::msg::LegCommands>::SharedPtr FL_leg_commands_subscription_;
    rclcpp::Subscription<custom_interfaces::msg::LegCommands>::SharedPtr FR_leg_commands_subscription_;
    rclcpp::Subscription<custom_interfaces::msg::LegCommands>::SharedPtr BL_leg_commands_subscription_;
    rclcpp::Subscription<custom_interfaces::msg::LegCommands>::SharedPtr BR_leg_commands_subscription_;
    
    
    rclcpp::Publisher<custom_interfaces::msg::Leds>::SharedPtr led_publisher_;
    
    
    
    // Variables to store the current state
    int state_ = 0;
    int control_mode_ = 4;
    float FL_position_cmds_[3];
    int FL_position_cmds_speeds_[3];
    float FR_position_cmds_[3];
    int FR_position_cmds_speeds_[3];
    float BL_position_cmds_[3];
    int BL_position_cmds_speeds_[3];
    float BR_position_cmds_[3];
    int BR_position_cmds_speeds_[3];
    
    float torque_cmds_[3];
    const char* interface_FL = "can1"; // Example interface name
    const char* interface_FR = "can0"; // Example interface name
    const char* interface_BL = "can3"; // Example interface name
    const char* interface_BR = "can2"; // Example interface name
    
    Leg FL_leg_memory;
    Leg FR_leg_memory;
    Leg BL_leg_memory;
    Leg BR_leg_memory;
    
    Leg_commands FL_commands;
    Leg_commands FR_commands;
    Leg_commands BL_commands;
    Leg_commands BR_commands;
    
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
        
        
		restart_commands(FL_commands);
        restart_commands(FR_commands);
        restart_commands(BL_commands);
        restart_commands(BR_commands);
        
       
    
    
        leg_readings_publisher_FL_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_FL", 10);
        leg_readings_publisher_FR_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_FR", 10);
        leg_readings_publisher_BL_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_BL", 10);
        leg_readings_publisher_BR_leg_ = this->create_publisher<custom_interfaces::msg::LegReadings>("leg_readings_BR", 10);
        
        // Publisher for "led_state" topic
        led_publisher_ = this->create_publisher<custom_interfaces::msg::Leds>("led_state", 10);

        // Subscriber for "buttons_state" topic
        control_interface_subscription_ = this->create_subscription<custom_interfaces::msg::Buttons>(
            "buttons_state", 10, std::bind(&ButtonProcessorNode::buttons_state_callback, this, std::placeholders::_1));
        
        FL_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>("FL_leg_commands",
        10,[&](const custom_interfaces::msg::LegCommands::SharedPtr msg) {leg_commands_callback(msg, FL_commands);
        }); 
        
        FR_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>("FR_leg_commands",
        10,[&](const custom_interfaces::msg::LegCommands::SharedPtr msg) {leg_commands_callback(msg, FR_commands);
        }); 
        
        BL_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>("BL_leg_commands",
        10,[&](const custom_interfaces::msg::LegCommands::SharedPtr msg) {leg_commands_callback(msg, BL_commands);
        }); 
        
        BR_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>("BR_leg_commands",
        10,[&](const custom_interfaces::msg::LegCommands::SharedPtr msg) {leg_commands_callback(msg, BR_commands);
        }); 
		
        //FR_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>(
		//"FR_leg_commands", 10,
		//std::bind(&ButtonProcessorNode::leg_commands_callback, this, std::placeholders::_1, std::ref(&FR_commands)));
		
		//BL_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>(
		//"BL_leg_commands", 10,
		//std::bind(&ButtonProcessorNode::leg_commands_callback, this, std::placeholders::_1, std::ref(&BL_commands)));
		
		//BR_leg_commands_subscription_ = this->create_subscription<custom_interfaces::msg::LegCommands>(
		//"BR_leg_commands", 10,
		//std::bind(&ButtonProcessorNode::leg_commands_callback, this, std::placeholders::_1, std::ref(&BR_commands)));
            
        // Run the switch-case logic continuously in separate threads
        std::cout << "NETWORKS STARTED" << std::endl;
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_FL_), std::ref(leg_readings_publisher_FL_leg_), std::ref(FL_leg_memory), std::ref(FL_commands)).detach();
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_FR_), std::ref(leg_readings_publisher_FR_leg_), std::ref(FR_leg_memory), std::ref(FR_commands)).detach();
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_BL_), std::ref(leg_readings_publisher_BL_leg_), std::ref(BL_leg_memory), std::ref(BL_commands)).detach();
        std::thread(&ButtonProcessorNode::process, this, std::ref(legCAN_BR_), std::ref(leg_readings_publisher_BR_leg_), std::ref(BR_leg_memory), std::ref(BR_commands)).detach();
        
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(FL_leg_memory), std::ref(leg_readings_publisher_FL_leg_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(FR_leg_memory), std::ref(leg_readings_publisher_FR_leg_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(BL_leg_memory), std::ref(leg_readings_publisher_BL_leg_)).detach();
        std::thread(&ButtonProcessorNode::publish_leg_reading, this, std::ref(BR_leg_memory), std::ref(leg_readings_publisher_BR_leg_)).detach();
        
    }
private:

    void process(Leg_CAN_Network legCAN, rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr publisher, Leg& leg_memory, Leg_commands& Leg_cmd) {
        rclcpp::Rate rate(500); // 500 Hz
        auto last_pub_time = this->now();
                
        
        
        while (rclcpp::ok()) {
            
            //std::cout << "FAILED: " << legCAN.leg.failed_messages << std::endl;     
            
            leg_memory = legCAN.leg;
            if (legCAN.leg.failed_messages >= 2){
				legCAN.RESTART_Network();
				legCAN.leg.failed_messages = 0;}
			else{
				legCAN.runCommunication(state_, Leg_cmd.control_mode, Leg_cmd.joint_angles, Leg_cmd.joint_speeds, Leg_cmd.joint_torques);}
				
            rate.sleep();
        }
    }
    
    
    
    void publish_leg_reading(Leg& leg_memory, rclcpp::Publisher<custom_interfaces::msg::LegReadings>::SharedPtr publisher) {
        
        rclcpp::Rate rate(500);
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
    
   
    
    void leg_commands_callback(const custom_interfaces::msg::LegCommands::SharedPtr msg, Leg_commands& leg_commands) {
        // Update the state variables based on the callback
        leg_commands.control_mode = msg->control_mode;
		std::copy(msg->joint_angles.begin(), msg->joint_angles.begin() + 3, leg_commands.joint_angles);
		std::copy(msg->joint_speed.begin(), msg->joint_speed.begin() + 3, leg_commands.joint_speeds);
		std::copy(msg->joint_torques.begin(), msg->joint_torques.begin() + 3, leg_commands.joint_torques);
        //std::cout << "Should 1: " << leg_commands.joint_angles[0] << std::endl; 
    }
    
    void restart_commands(Leg_commands& Leg_cmd){
		Leg_cmd.control_mode = 2;  
    
		Leg_cmd.joint_angles[0] = 0.0f;          
		Leg_cmd.joint_angles[1] = 60.75f;
		Leg_cmd.joint_angles[2] = -121.50f;
		
		Leg_cmd.joint_speeds[0] = 1;
		Leg_cmd.joint_speeds[1] = 1;
		Leg_cmd.joint_speeds[2] = 1;
		
		Leg_cmd.joint_torques[0] = 0.0f;
		Leg_cmd.joint_torques[1] = 0.0f;
		Leg_cmd.joint_torques[2] = 0.0f;	
	};
	
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ButtonProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
