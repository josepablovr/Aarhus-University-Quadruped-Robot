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

float torques[3];
float speeds[3];
float encoders[3];
float corrected_torques[3];
float corrected_speeds[3];
float corrected_encoders[3];    
float multi_encoders[3];

void updateReadings(Leg FL) {
    torques[0] = FL.shoulder.torque_current;
    torques[1] = FL.hip.torque_current;
    torques[2] = FL.knee.torque_current;
    
    speeds[0] = FL.shoulder.speed;
    speeds[1] = FL.hip.speed;
    speeds[2] = FL.knee.speed;
    
    encoders[0] = FL.shoulder.angular_position;
    encoders[1] = FL.hip.angular_position;
    encoders[2] = FL.knee.angular_position;
    
    corrected_torques[0] = FL.shoulder.joint_torque;
    corrected_torques[1] = FL.hip.joint_torque;
    corrected_torques[2] = FL.knee.joint_torque;
    
    corrected_speeds[0] = FL.shoulder.joint_speed;
    corrected_speeds[1] = FL.hip.joint_speed;
    corrected_speeds[2] = FL.knee.joint_speed;
    
    corrected_encoders[0] = FL.shoulder.joint_angle;
    corrected_encoders[1] = FL.hip.joint_angle;
    corrected_encoders[2] = FL.knee.joint_angle;
    
    multi_encoders[0] = FL.shoulder.multi_angle_position;
    multi_encoders[1] = FL.hip.multi_angle_position;
    multi_encoders[2] = FL.knee.multi_angle_position;}
    
void interfaceThread(const char* interface, double& totalTime, int& cycleCount) {
    cycleCount = 1010;
    int failed_cycles = 0;
    int msgs_received = 0;
    // Bring up the CAN interface with the specified bitrate
    std::string command = "sudo ip link set dev " + std::string(interface) + " up type can bitrate 1000000";
    int result = system(command.c_str());

    if (result != 0) {
        std::cerr << "Failed to bring up CAN interface " << interface << std::endl;
        std::cout << "Restarting the interface " << interface << std::endl;
        system(("sudo ip link set dev " + std::string(interface) + " down").c_str());
        sleep(0.1); // Wait for 0.1 second before bringing the interface up again
        result = system(command.c_str());
        if (result != 0) {
            std::cerr << "Failed to restart CAN interface " << interface << std::endl;
            return;
        }
    }

    std::cout << "Successfully Started" << std::endl;

    int s = setupCANSocket(interface);
    unsigned char data_frame[8];
    int debug = 0;
    Motor_Status motor1;
    motor1.id = 0x141;
    Motor_Status motor2;
    motor2.id = 0x142;
    Motor_Status motor3;
    motor3.id = 0x143;
    
    Leg FL;
    FL.leg_index = 0;
    FL.shoulder = motor1;
    FL.hip = motor2;
    FL.knee = motor3;
    
  
    
    auto start = std::chrono::steady_clock::now();
    double max_frequency = 10000.0; // For example, 10000 cycles per second
    std::chrono::steady_clock::time_point start_time, end_time;
    double max_cycle_duration = 1.0 / max_frequency; // Maximum duration of one cycle
    end_time = std::chrono::steady_clock::now();
    double time_diff = 0;

    // Open the file for writing
    std::string filename = "Communication_Test_CPP_" + std::string(interface) + ".txt";
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    int i = 1;
    int msg_count = 0;
    
    //CHECK IF THE NETWORK IS WORKING
    command_read_multi_turn_angle(data_frame);        
    sendMessage(s, motor1.id, data_frame, 8, 0);
    if (receiveMessage2(s, &FL, debug) == 0) {           
        msgs_received += 1;            
    } else {
        std::cerr << "Failed to Receive CAN Messages" << std::endl;
    }
    sendMessage(s, motor2.id, data_frame, 8, 0);    
    if (receiveMessage2(s, &FL, debug) == 0) {
        msgs_received += 1;            
    } else {
        std::cerr << "Failed to Receive CAN Messages" << std::endl;
    } 
    sendMessage(s, motor3.id, data_frame, 8, 0);
    if (receiveMessage2(s, &FL, debug) == 0) {           
        msgs_received += 1;     
    } else {
        std::cerr << "Failed to Receive CAN Messages" << std::endl;
    }       
    if (msgs_received != 3){
        failed_cycles +=1;}
    msgs_received = 0;
    
    command_read_motor_status_2(data_frame); 
    sendMessage(s, motor1.id, data_frame, 8, 0);
    if (receiveMessage2(s, &FL, debug) == 0) {           
        msgs_received += 1;            
    } else {
        std::cerr << "Failed to Receive CAN Messages" << std::endl;
    }
    sendMessage(s, motor2.id, data_frame, 8, 0);    
    if (receiveMessage2(s, &FL, debug) == 0) {
        msgs_received += 1;            
    } else {
        std::cerr << "Failed to Receive CAN Messages" << std::endl;
    } 
    sendMessage(s, motor3.id, data_frame, 8, 0);
    if (receiveMessage2(s, &FL, debug) == 0) {           
        msgs_received += 1;     
    } else {
        std::cerr << "Failed to Receive CAN Messages" << std::endl;
    }    
    
    
      
    EncoderCounter shoulder_Encoder;
    EncoderCounter hip_Encoder;
    EncoderCounter knee_Encoder;
    
    EncoderCounter Encoders[3] = {shoulder_Encoder, hip_Encoder, knee_Encoder};
    shoulder_Encoder.start(FL.shoulder.multi_angle_position, FL.shoulder.single_angle_position);
    hip_Encoder.start(FL.hip.multi_angle_position, FL.shoulder.single_angle_position);
    knee_Encoder.start(FL.knee.multi_angle_position, FL.shoulder.single_angle_position);
    
    
    ControlMode mode;
    mode = Read_Status;
    
    //Readings
    
    float published_torques_command[3] = {0.0,0.0,0.0}; //Nm
    float published_position_command[3] = {0.0,0.0,0.0}; //deg
    int published_speed_pos_ctr[3] = {0,0,0}; //deg
        
    float torques_command[3] = {0.0,0.0,0.0}; //Nm
    float position_command[3] = {0.0,0.0,0.0}; //deg
    int speed_pos_ctr[3] = {0,0,0}; //deg/s
    
    int published_mode_numb = 0;
    // - Case 1 returns the enum value Read_Multi_Turn
	// - Case 2 returns the enum value Read_Status
	// - Case 3 returns the enum value Torque_Control
	// - Case 4 returns the enum value Position_Control
    
    
    
    int published_state = 1;    
    // - Case 0 returns the enum value IDLE
	// - Case 1 returns the enum value STOP
	// - Case 2 returns the enum value START
	// - Case 3 returns the enum value PAUSE
	// - Case 4 returns the enum value RESTART
	// - Case 5 returns the enum value EXTRA

    
    ControlMode published_mode = intToMode(published_mode_numb);
    
    MainState Network_State = intToState(published_state);
    
    
	float restart_pos[3] = {0.00, 60.75, -121.50};
    int restart_speed = 10;
    
    while (i <= cycleCount) {
        start_time = std::chrono::steady_clock::now();
        
        switch (Network_State) {
			case IDLE:
				mode = Read_Status;
				
				break;
			case STOP:
			
				mode = Torque_Control;
				
				for (int i = 0; i < 3; ++i) {
					torques_command[i] = 0.0;					
				}
				
				break;
			case START:
				mode = published_mode;	
				for (int i = 0; i < 3; ++i) {
					torques_command[i] = published_torques_command[i];
					position_command[i] = published_position_command[i];
					speed_pos_ctr[i] = published_speed_pos_ctr[i];
				}
				
				break;
			case PAUSE:
				mode = Position_Control;
				for (int i = 0; i < 3; ++i) {
					position_command[i] = restart_pos[i];
					speed_pos_ctr[i] = 0;
				}
			
			
				break;
			case RESTART:
			
				mode = Position_Control;
				
				for (int i = 0; i < 3; ++i) {
					position_command[i] = restart_pos[i];
					speed_pos_ctr[i] = restart_speed;
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
				command_read_multi_turn_angle(data_frame);   
				 
				//SEND MSGS                 
				sendMessage(s, motor1.id, data_frame, 8, 0);
				sendMessage(s, motor2.id, data_frame, 8, 0);
				if (receiveMessage2(s, &FL, debug) == 0)         
					msgs_received += 1;                    
				if (receiveMessage2(s, &FL, debug) == 0) 
					msgs_received += 1; 
				sendMessage(s, motor3.id, data_frame, 8, 0);
				if (receiveMessage2(s, &FL, debug) == 0)          
					msgs_received += 1;     
				
				//CORRECT
				updateReadings(FL);
				angle_corrections(FL.leg_index, multi_encoders, corrected_encoders);
				std::cerr << corrected_encoders[1]<< std::endl;
			 
				break;
			case Read_Status:
				command_read_motor_status_2(data_frame);     
				
				//SEND MSGS                 
				sendMessage(s, motor1.id, data_frame, 8, 0);
				sendMessage(s, motor2.id, data_frame, 8, 0);
				if (receiveMessage2(s, &FL, debug) == 0)         
					msgs_received += 1;                    
				if (receiveMessage2(s, &FL, debug) == 0) 
					msgs_received += 1; 
				sendMessage(s, motor3.id, data_frame, 8, 0);
				if (receiveMessage2(s, &FL, debug) == 0)          
					msgs_received += 1;   
				
				
				//CORRECT  
				updateReadings(FL);
				FL.shoulder.angular_position = shoulder_Encoder.get_multi_turn(FL.shoulder.single_angle_position);
				FL.hip.angular_position = hip_Encoder.get_multi_turn(FL.hip.single_angle_position);
				FL.knee.angular_position = knee_Encoder.get_multi_turn(FL.knee.single_angle_position);              
				
			  
				
				
				readings_corrections(FL.leg_index, torques, speeds, encoders,
						  corrected_torques, corrected_speeds, corrected_encoders);
				std::cerr << corrected_encoders[0]<< std::endl; 
				
						
						  
				break;
			case Torque_Control:
				real_robot_commands_torques(FL.leg_index, torques_command);
				//std::cerr << torques_command[2]<< std::endl;
				
				//SEND MSGS   
				command_torque_control(data_frame, torques_command[0]);              
				sendMessage(s, motor1.id, data_frame, 8, 0);
				
				command_torque_control(data_frame, torques_command[1]);
				sendMessage(s, motor2.id, data_frame, 8, 0);
				if (receiveMessage2(s, &FL, debug) == 0)         
					msgs_received += 1;                    
				if (receiveMessage2(s, &FL, debug) == 0) 
					msgs_received += 1; 
				command_torque_control(data_frame, torques_command[2]);
				sendMessage(s, motor3.id, data_frame, 8, 0);
				if (receiveMessage2(s, &FL, debug) == 0)          
					msgs_received += 1;   
			   
				//CORRECTION   
				updateReadings(FL);
				FL.shoulder.angular_position = shoulder_Encoder.get_multi_turn(FL.shoulder.single_angle_position);
				FL.hip.angular_position = hip_Encoder.get_multi_turn(FL.hip.single_angle_position);
				FL.knee.angular_position = knee_Encoder.get_multi_turn(FL.knee.single_angle_position);              
				
				
				readings_corrections(FL.leg_index, torques, speeds, encoders,
						  corrected_torques, corrected_speeds, corrected_encoders);
				
				break;
				
			case Position_Control:
			
				if(Security_Position_Joint(position_command) == 1){
			
					real_robot_commands_angles(FL.leg_index, position_command);
					
					
					//std::cerr << position_command[2]<< std::endl;
					//SEND MSGS       
					command_position_control_2(data_frame, position_command[0], speed_pos_ctr[0]);         
					//std::cerr << position_command[0]<< std::endl; 
					
					sendMessage(s, motor1.id, data_frame, 8, 0);
					
			
				
					
					command_position_control_2(data_frame, position_command[1], speed_pos_ctr[1]);
					sendMessage(s, motor2.id, data_frame, 8, 0);
					if (receiveMessage2(s, &FL, debug) == 0)         
						msgs_received += 1;                    
					if (receiveMessage2(s, &FL, debug) == 0) 
						msgs_received += 1; 
					command_position_control_2(data_frame, position_command[2], speed_pos_ctr[2]);
					sendMessage(s, motor3.id, data_frame, 8, 0);
					if (receiveMessage2(s, &FL, debug) == 0)          
						msgs_received += 1;   
					
					
					//CORRECTIONS
					updateReadings(FL);
					FL.shoulder.angular_position = shoulder_Encoder.get_multi_turn(FL.shoulder.single_angle_position);
					FL.hip.angular_position = hip_Encoder.get_multi_turn(FL.hip.single_angle_position);
					FL.knee.angular_position = knee_Encoder.get_multi_turn(FL.knee.single_angle_position);              
					
					readings_corrections(FL.leg_index, torques, speeds, encoders,
							  corrected_torques, corrected_speeds, corrected_encoders);}
				
				
				break;
			default:
				printf("Invalid mode\n");
				break;
	}
           
	   
            
            
            
        
        

           
        
        
        
        
        
        
        
        
        if (msgs_received != 3){
            failed_cycles +=1;}
        msgs_received = 0;
        
        
        
        //std::cerr << FL.hip.angular_position << std::endl;
        //std::cerr << FL.hip.multi_angle_position << std::endl;
        //std::cerr << FL.knee.multi_angle_position << std::endl;
        
        
       
        end_time = std::chrono::steady_clock::now(); // Record end time of the cycle
        if (i>10){
            
            time_diff = std::chrono::duration_cast<std::chrono::duration<double>>(end_time-start_time).count();
            double frequency = 1.0 / time_diff;
            outFile << std::fixed << std::setprecision(3) << frequency << std::endl;}
        
        
        i += 1;
        //std::cerr << i << std::endl;
        
    }
    std::cerr << failed_cycles << std::endl;
    // Close the file
    outFile.close();

    auto end = std::chrono::steady_clock::now();
    totalTime = std::chrono::duration<double>(end - start).count();
}
int main() {
    
    const char* interfaces[] = {"can1", "can0", "can2", "can3"};
    
    std::vector<std::thread> threads;
    std::vector<double> totalTimes(4, 0);
    std::vector<int> cycleCounts(4, 0);

    // Create and start threads
    for (int i = 0; i < 1; ++i) {
        threads.emplace_back(interfaceThread, interfaces[i], std::ref(totalTimes[i]), std::ref(cycleCounts[i]));
    }

    // Join threads
    for (auto& thread : threads) {
        thread.join();
    }

    // Calculate and print average frequency for each thread
    for (int i = 0; i < 4; ++i) {
        double avgFrequency = cycleCounts[i] / totalTimes[i];
        std::cout << "Interface " << interfaces[i] << " average frequency: " << avgFrequency << " cycles per second" << std::endl;
    }

    return 0;
}
