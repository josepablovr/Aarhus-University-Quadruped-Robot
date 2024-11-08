#include "comm_interface/Leg_CAN_Network.h"
#include "comm_interface/CAN_BUS.h"

// Constructor


MainState intToState(int state) {
    switch (state) {
        case 0:
            return IDLE;
        case 1:
            return STOP;
        case 2:
            return START;
        case 3:
            return PAUSE;
        case 4:
            return RESTART;
        case 5:
            return EXTRA;
        default:
            return IDLE; // or any default state you prefer
    }
}



Leg_CAN_Network::Leg_CAN_Network(const char* interface) : shoulder_Encoder(), hip_Encoder(), knee_Encoder() {
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
    leg.failed_messages = 0;
    // Set leg IDs
    leg.shoulder.id = 0x141;
    leg.hip.id = 0x142;
    leg.knee.id = 0x143;
    
    // Setup sockets
    Setup_Network();
    // Call interface setup function
    s = setupCANSocket(interface);
    //CHECK IF THE NETWORK IS WORKING
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
Leg_CAN_Network::~Leg_CAN_Network() {
    // Perform any necessary cleanup
}

// Function to run communication based on robot state and control mode
void Leg_CAN_Network::runCommunication(int state, int control_Mode, float publishedPositionCommand[3],
                          int publishedSpeedPosCtr[3], float publishedTorquesCommand[3]) {
    robotState = intToState(state);
    controlMode = intToMode(control_Mode);
    //std::cout << "Network Cycle" << std::endl;
    float joint_commands[3];
    joint_commands[0] = publishedPositionCommand[0];
    joint_commands[1] = publishedPositionCommand[1];
    joint_commands[2] = publishedPositionCommand[2];
    
    int speed_commands[3];
    speed_commands[0] = publishedSpeedPosCtr[0];
    speed_commands[1] = publishedSpeedPosCtr[1];
    speed_commands[2] = publishedSpeedPosCtr[2];
    
    float torque_commands[3];
    torque_commands[0] = publishedTorquesCommand[0];
    torque_commands[1] = publishedTorquesCommand[1];
    torque_commands[2] = publishedTorquesCommand[2];
    
    switch (robotState) {
        case IDLE:
            mode = Read_Status;              
            break;
        case STOP:          
            mode = Torque_Control;              
            for (int i = 0; i < 3; ++i) {
                torque_commands[i] = 0.0;
            }               
            break;
        case START:
            mode = controlMode;                
            for (int i = 0; i < 3; ++i) {
                //joint_commands[i] = standing_pos_1[i];
                //speed_commands[i] = restart_speed;
            }   
            break;   
            
        case PAUSE:
            mode = Position_Control;
            for (int i = 0; i < 3; ++i) {
                joint_commands[i] = restart_pos[i];
                speed_commands[i] = 1;
            }           
            break;
        case RESTART:           
            mode = Position_Control;                
            for (int i = 0; i < 3; ++i) {
                joint_commands[i] = restart_pos[i];
                speed_commands[i] = restart_speed;
            }   
            break;                          
        case EXTRA:           
            mode = Position_Control;                
            for (int i = 0; i < 3; ++i) {
                joint_commands[i] = standing_pos_2[i];
                speed_commands[i] = restart_speed;
            }   
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
            Update_Leg_status();
            //std::cerr << corrected_encoders[2]<< std::endl; 
            //std::cerr << encoders[2]<< std::endl;
            //std::cerr << leg.knee.multi_angle_position<< std::endl;
            //std::cerr << "END" << std::endl; 
            break;
        case Torque_Control:
            
            if (get_Torque_Control(s, &leg, torque_commands) == 0)
                  std::cerr << "Failed to Receive CAN Messages" << std::endl;   
            
            updateReadings(leg);
            Update_Encoders(&leg, Encoders);            
            readings_corrections(leg.leg_index, torques, speeds, encoders,
                    corrected_torques, corrected_speeds, corrected_encoders);  
            Update_Leg_status();
                    
            break;
        case Position_Control:
            if (get_Position_Control(s, &leg, joint_commands, speed_commands) == 0)
                std::cerr << "Failed to Receive CAN Messages" << std::endl;   
                   
            updateReadings(leg);
            Update_Encoders(&leg, Encoders);                 
            readings_corrections(leg.leg_index, torques, speeds, encoders,
                        corrected_torques, corrected_speeds, corrected_encoders);  
            Update_Leg_status();              
            break;
        default:
            printf("Invalid mode\n");
            break;
    }
}

// Function to update readings

void Leg_CAN_Network::updateReadings(Leg leg) {
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
    
    //corrected_torques[0] = leg.shoulder.joint_torque;
    //corrected_torques[1] = leg.hip.joint_torque;
    //corrected_torques[2] = leg.knee.joint_torque;
    
    //corrected_speeds[0] = leg.shoulder.joint_speed;
    //corrected_speeds[1] = leg.hip.joint_speed;
    //corrected_speeds[2] = leg.knee.joint_speed;
    
    //corrected_encoders[0] = leg.shoulder.joint_angle;
    //corrected_encoders[1] = leg.hip.joint_angle;
    //corrected_encoders[2] = leg.knee.joint_angle;
    
    
    //leg.shoulder.joint_torque = corrected_torques[0];
    //leg.hip.joint_torque = corrected_torques[1];
    //leg.knee.joint_torque = corrected_torques[2];

    //leg.shoulder.joint_speed = corrected_speeds[0];
    //leg.hip.joint_speed = corrected_speeds[1];
    //leg.knee.joint_speed = corrected_speeds[2];
    
    
    
    //leg.shoulder.joint_angle = corrected_encoders[0];
    //leg.hip.joint_angle = corrected_encoders[1]; 
    //leg.knee.joint_angle = corrected_encoders[2];
    
    multi_encoders[0] = leg.shoulder.multi_angle_position;
    multi_encoders[1] = leg.hip.multi_angle_position;
    multi_encoders[2] = leg.knee.multi_angle_position;
}


void Leg_CAN_Network::Update_Leg_status(){
    leg.shoulder.joint_torque = corrected_torques[0];
    leg.hip.joint_torque = corrected_torques[1];
    leg.knee.joint_torque = corrected_torques[2];

    leg.shoulder.joint_speed = corrected_speeds[0];
    leg.hip.joint_speed = corrected_speeds[1];
    leg.knee.joint_speed = corrected_speeds[2];
    
    
    
    leg.shoulder.joint_angle = corrected_encoders[0];
    leg.hip.joint_angle = corrected_encoders[1]; 
    leg.knee.joint_angle = corrected_encoders[2];
}

void Leg_CAN_Network::Setup_Network(){
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


void Leg_CAN_Network::RESTART_Network(){
    // Bring up the CAN interface with the specified bitrate
    std::string command = "sudo ip link set dev " + std::string(interface_name) + " up type can bitrate 1000000";
    int result = system(command.c_str());

    if (result != 0) {
        //std::cerr << "Failed to bring up CAN interface " << interface_name << std::endl;
        //std::cout << "Restarting the interface " << interface_name << std::endl;
        //system(("sudo ip link set dev " + std::string(interface_name) + " down").c_str());
        
        result = system(command.c_str());
        if (result != 0) {
            std::cerr << "Failed to restart CAN interface " << interface_name << std::endl;
            return;
        }
    }
    std::cout << "Successfully Restarted" << std::endl;
    
    s = setupCANSocket(interface_name);
}


void Leg_CAN_Network::Update_Encoders(Leg* leg, EncoderCounter* Encoders){
    leg->shoulder.angular_position = Encoders[0].get_multi_turn(leg->shoulder.single_angle_position);
    leg->hip.angular_position = Encoders[1].get_multi_turn(leg->hip.single_angle_position);
    leg->knee.angular_position = Encoders[2].get_multi_turn(leg->knee.single_angle_position); 
}

void Leg_CAN_Network::Start_Encoders(Leg*leg, EncoderCounter* Encoders){
    Encoders[0].start(leg->shoulder.multi_angle_position,leg->shoulder.single_angle_position);
    Encoders[1].start(leg->hip.multi_angle_position,leg->hip.single_angle_position);
    Encoders[2].start(leg->knee.multi_angle_position,leg->knee.single_angle_position);
}
