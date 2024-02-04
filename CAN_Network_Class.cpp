#include <iostream>
#include <fstream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <vector>
#include "RMD_X8.h"
#include "CAN_BUS.h"
#include <unistd.h> // For sleep
#include <iomanip>
#include "EncoderCounter.h"

// Define Leg_CAN_Network class
class Leg_CAN_Network {
private:
    // Define private member variables
    float torques[3];
    float speeds[3];
    float encoders[3];
    float corrected_torques[3];
    float corrected_speeds[3];
    float corrected_encoders[3];
    float multi_encoders[3];
    int legIndex; // Store the leg index
    int s; //Socket
    Leg leg;
    ControlMode mode;
    float restart_pos[3] = {0.00, 60.75, -121.50};
    int restart_speed = 10;
    EncoderCounter shoulder_Encoder;
    EncoderCounter hip_Encoder;
    EncoderCounter knee_Encoder;
    EncoderCounter Encoders[3] = {shoulder_Encoder, hip_Encoder, knee_Encoder};
    const char* interface_name;

    // Other private member variables
    // Add other private member variables as needed

public:
    // Constructor
    Leg_CAN_Network(const char* interface) : shoulder_Encoder(), hip_Encoder(), knee_Encoder() {
        interface_name = interface;
        // Set leg index based on interface name
        if (std::string(interface) == "can0") {
            legIndex = 1;
        } else if (std::string(interface) == "can1") {
            legIndex = 0;
        } else if (std::string(interface) == "can2") {
            legIndex = 3;
        } else if (std::string(interface) == "can3") {
            legIndex = 2;
        } else {
            // Default to leg index 0 if interface name is not recognized
            legIndex = 0;
        }
        leg.leg_index = legIndex;
        // Set leg IDs
        leg.shoulder.id = 0x141;
        leg.hip.id = 0x142;
        leg.knee.id = 0x143;
        // Setup sockets
        Setup_Network();
        // Call interface setup function
        s = setupCANSocket(interface);
        // CHECK IF THE NETWORK IS WORKING
        if (get_Multi_turn_angle(s, &leg) == 0)
            std::cerr << "Failed to Receive CAN Messages" << std::endl;
        else
            std::cerr << "CAN Network is working" << std::endl;
            
        if (get_Read_Status(s, &leg) == 0)
            std::cerr << "Failed to Receive CAN Messages" << std::endl;
        // Initialize other member variables as needed
        Start_Encoders(&leg,Encoders);
    }

    // Destructor
    ~Leg_CAN_Network() {
        // Perform any necessary cleanup
    }

    // Function to run communication based on robot state and control mode
    void runCommunication(MainState robotState, ControlMode controlMode, float publishedPositionCommand[3],
                          int publishedSpeedPosCtr[3], float publishedTorquesCommand[3]) {
        
        switch (robotState) {
            case IDLE:
                mode = Read_Status;              
                break;
            case STOP:          
                mode = Torque_Control;              
                for (int i = 0; i < 3; ++i) {
                    publishedTorquesCommand[i] = 0.0;
                }               
                break;
            case START:
                mode = controlMode; 
                for (int i = 0; i < 3; ++i) {
                    publishedTorquesCommand[i] = publishedTorquesCommand[i];
                    publishedPositionCommand[i] = publishedPositionCommand[i];
                    publishedSpeedPosCtr[i] = publishedSpeedPosCtr[i];
                }               
                break;
            case PAUSE:
                mode = Position_Control;
                for (int i = 0; i < 3; ++i) {
                    publishedPositionCommand[i] = restart_pos[i];
                    publishedSpeedPosCtr[i] = 0;
                }           
                break;
            case RESTART:           
                mode = Position_Control;                
                for (int i = 0; i < 3; ++i) {
                    publishedPositionCommand[i] = restart_pos[i];
                    publishedSpeedPosCtr[i] = restart_speed;
                }   
                break;                          
            case EXTRA:           
                mode = Read_Status;                    
                break;              
            default:
                std::cout << "Unknown state" << std::endl;
        }

        switch (mode) {
            case Read_Multi_Turn:                 
                if (get_Multi_turn_angle(s, &leg) == 0)
                    std::cerr << "Failed to Receive CAN Messages" << std::endl;          
                updateReadings(leg);
                angle_corrections(leg.leg_index, multi_encoders, corrected_encoders);
                std::cerr << leg.shoulder.multi_angle_position<< std::endl;        
                break;
            case Read_Status:
                if (get_Read_Status(s, &leg) == 0)
                    std::cerr << "Failed to Receive CAN Messages" << std::endl;           
                                      
                Update_Encoders(&leg, Encoders);
                updateReadings(leg);  
                readings_corrections(leg.leg_index, torques, speeds, encoders,
                corrected_torques, corrected_speeds, corrected_encoders);
                std::cerr << corrected_encoders[2]<< std::endl; 
                std::cerr << encoders[2]<< std::endl;
                std::cerr << leg.knee.multi_angle_position<< std::endl;
                std::cerr << "END" << std::endl; 
                break;
            case Torque_Control:
                
                if (get_Torque_Control(s, &leg, publishedTorquesCommand) == 0)
                    std::cerr << "Failed to Receive CAN Messages" << std::endl;   
                
                updateReadings(leg);
                Update_Encoders(&leg, Encoders);            
                readings_corrections(leg.leg_index, torques, speeds, encoders,
                        corrected_torques, corrected_speeds, corrected_encoders);                
                break;
            case Position_Control:
                if (get_Position_Control(s, &leg, publishedPositionCommand, publishedSpeedPosCtr) == 0)
                    std::cerr << "Failed to Receive CAN Messages" << std::endl;   
                       
                updateReadings(leg);
                Update_Encoders(&leg, Encoders);                 
                readings_corrections(leg.leg_index, torques, speeds, encoders,
                            corrected_torques, corrected_speeds, corrected_encoders);                
                break;
            default:
                printf("Invalid mode\n");
                break;
        }
    }

private:
  
    // Function to update readings
    void updateReadings(Leg leg) {
        // Update readings as before
        torques[0] = leg.shoulder.torque_current;
        torques[1] = leg.hip.torque_current;
        torques[2] = leg.knee.torque_current;
        
        speeds[0] = leg.shoulder.speed;
        speeds[1] = leg.hip.speed;
        speeds[2] = leg.knee.speed;
        
        encoders[0] = leg.shoulder.angular_position;
        encoders[1] = leg.hip.angular_position;
        encoders[2] = leg.knee.angular_position;
        
        corrected_torques[0] = leg.shoulder.joint_torque;
        corrected_torques[1] = leg.hip.joint_torque;
        corrected_torques[2] = leg.knee.joint_torque;
        
        corrected_speeds[0] = leg.shoulder.joint_speed;
        corrected_speeds[1] = leg.hip.joint_speed;
        corrected_speeds[2] = leg.knee.joint_speed;
        
        corrected_encoders[0] = leg.shoulder.joint_angle;
        corrected_encoders[1] = leg.hip.joint_angle;
        corrected_encoders[2] = leg.knee.joint_angle;
        
        multi_encoders[0] = leg.shoulder.multi_angle_position;
        multi_encoders[1] = leg.hip.multi_angle_position;
        multi_encoders[2] = leg.knee.multi_angle_position;
    }

    void Setup_Network(){
        // Bring up the CAN interface with the specified bitrate
        std::string command = "sudo ip link set dev " + std::string(interface_name) + " up type can bitrate 1000000";
        int result = system(command.c_str());

        if (result != 0) {
            std::cerr << "Failed to bring up CAN interface " << interface_name << std::endl;
            std::cout << "Restarting the interface " << interface_name << std::endl;
            system(("sudo ip link set dev " + std::string(interface_name) + " down").c_str());
            
            result = system(command.c_str());
            if (result != 0) {
                std::cerr << "Failed to restart CAN interface " << interface_name << std::endl;
                return;
            }
        }
        std::cout << "Successfully Started" << std::endl;
    }

    void Update_Encoders(Leg* leg, EncoderCounter* Encoders){
        leg->shoulder.angular_position = Encoders[0].get_multi_turn(leg->shoulder.single_angle_position);
        leg->hip.angular_position = Encoders[1].get_multi_turn(leg->hip.single_angle_position);
        leg->knee.angular_position = Encoders[2].get_multi_turn(leg->knee.single_angle_position); 
    }
    void Start_Encoders(Leg*leg, EncoderCounter* Encoders){
        Encoders[0].start(leg->shoulder.multi_angle_position,leg->shoulder.single_angle_position);
        Encoders[1].start(leg->hip.multi_angle_position,leg->hip.single_angle_position);
        Encoders[2].start(leg->knee.multi_angle_position,leg->knee.single_angle_position);
    }
    

    // Other private member functions
    // Add other private member functions as needed
};

int main() {
    // Example usage of the Leg_CAN_Network class
    const char* interface = "can1"; // Example interface name

    // Create an instance of Leg_CAN_Network
    Leg_CAN_Network legCAN(interface);

    // Example usage of runCommunication function
    MainState state = RESTART; // Example robot state
    ControlMode mode = Read_Status; // Example control mode
    float positionCommand[3] = {0.0, 0.0, 0.0}; // Example position command
    int speedPosCtr[3] = {0, 0, 0}; // Example speed/position counter
    float torquesCommand[3] = {0.0, 0.0, 0.0}; // Example torque command

    legCAN.runCommunication(state, mode, positionCommand, speedPosCtr, torquesCommand);
    legCAN.runCommunication(state, mode, positionCommand, speedPosCtr, torquesCommand);
    legCAN.runCommunication(state, mode, positionCommand, speedPosCtr, torquesCommand);
    return 0;
}
