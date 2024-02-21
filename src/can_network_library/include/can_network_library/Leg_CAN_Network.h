#ifndef LEG_CAN_NETWORK_H
#define LEG_CAN_NETWORK_H

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
    MainState robotState;
    ControlMode controlMode;

    // Other private member variables
    // Add other private member variables as needed

public:
    // Constructor
    Leg_CAN_Network(const char* interface);
    
    // Destructor
    ~Leg_CAN_Network();
    
    // Function to run communication based on robot state and control mode
    void runCommunication(int state, int control_Mode, float publishedPositionCommand[3],
                          int publishedSpeedPosCtr[3], float publishedTorquesCommand[3]);

private:
  
    // Function to update readings
    void updateReadings(Leg leg);

    void Setup_Network();

    void Update_Encoders(Leg* leg, EncoderCounter* Encoders);

    void Start_Encoders(Leg* leg, EncoderCounter* Encoders);
};

#endif // LEG_CAN_NETWORK_H
