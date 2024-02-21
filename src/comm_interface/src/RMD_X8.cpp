#include "comm_interface/RMD_X8.h"
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdlib.h>

#include <stdio.h>
#include <stdint.h> 
#include <string.h>
#include <stdbool.h>
#include <math.h>


void command_read_pid_RAM(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_PID_DATA;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_write_pid_RAM(unsigned char data_frame[8], int8_t  position_Kp, 
int8_t  position_Ki, int8_t  speed_Kp,int8_t  speed_Ki, int8_t  torque_Kp, 
int8_t  torque_Ki) {
    unsigned char can_data[8];
    can_data[0] = WRITE_PID_TO_RAM;
    can_data[1] = 0x00;
    can_data[2] = position_Kp;
    can_data[3] = position_Ki;
    can_data[4] = speed_Kp;
    can_data[5] = speed_Ki;
    can_data[6] = torque_Kp;
    can_data[7] = torque_Ki;
    memcpy(data_frame, &can_data, sizeof(unsigned char));

}


void command_write_pid_ROM(unsigned char data_frame[8], int8_t  position_Kp, 
int8_t  position_Ki, int8_t  speed_Kp,int8_t  speed_Ki, int8_t  torque_Kp, 
int8_t  torque_Ki) {
    unsigned char can_data[8];
    can_data[0] = WRITE_PID_TO_ROM;
    can_data[1] = 0x00;
    can_data[2] = position_Kp;
    can_data[3] = position_Ki;
    can_data[4] = speed_Kp;
    can_data[5] = speed_Ki;
    can_data[6] = torque_Kp;
    can_data[7] = torque_Ki;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_read_acceleration(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_ACCELERATION;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_write_acceleration(unsigned char data_frame[8], int32_t  acceleration) {
    unsigned char can_data[8];
    can_data[0] = WRITE_ACCELERATION;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = acceleration & 0xFF;
    can_data[5] = (acceleration >> 8) & 0xFF;
    can_data[6] = (acceleration >> 16) & 0xFF;
    can_data[7] = (acceleration >> 24) & 0xFF;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_read_encoder(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_ENCODER_DATA;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_write_encoder_offset(unsigned char data_frame[8], int16_t  offset) {
    unsigned char can_data[8];
    can_data[0] = WRITE_ENCODER_OFFSET;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = offset & 0xFF;
    can_data[7] = (offset >> 8) & 0xFF;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_write_encoder_zero(unsigned char data_frame[8], int16_t  zero) {
    unsigned char can_data[8];
    can_data[0] = WRITE_CURRENT_POSITION;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = zero & 0xFF;
    can_data[7] = (zero >> 8) & 0xFF;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_read_multi_turn_angle(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_MULTI_TURNS_ANGLE;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_read_single_turn_angle(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_SINGLE_ANGLE;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_read_motor_status_1(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_MOTOR_STATUS;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}
void command_clear_error_flag(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = CLEAR_MOTOR_ERROR_FLAG;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_read_motor_status_2(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_MOTOR_STATUS_2;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}
void command_read_motor_status_3(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = READ_MOTOR_STATUS_3;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}
void command_motor_OFF(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = MOTOR_OFF;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_motor_STOP(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = MOTOR_STOP;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_motor_RUN(unsigned char data_frame[8]) {
    unsigned char can_data[8];
    can_data[0] = MOTOR_RUNNING;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}
void NegToHex(int16_t num, char *hex_str) {
    snprintf(hex_str, 5, "%04X", (uint16_t)(-num));
}
void command_torque_control(unsigned char data_frame[8], int16_t torque_current) {
    
    
    if (torque_current > 2000 || torque_current < -2000)
        return;

    unsigned char can_data[8] = {TORQUE_CLOSED_LOOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    if (torque_current == 0)
        memcpy(data_frame, can_data, sizeof(unsigned char) * 8);
    else if (torque_current < 0) {
        char hex_torque[5];
        NegToHex(torque_current, hex_torque);
        can_data[5] = strtol(hex_torque, NULL, 16) & 0xFF;
        can_data[4] = (strtol(hex_torque, NULL, 16) >> 8) & 0xFF;
        memcpy(data_frame, can_data, sizeof(unsigned char) * 8);
    } else if (torque_current < 256) {
        can_data[4] = torque_current;
        memcpy(data_frame, can_data, sizeof(unsigned char) * 8);
    } else {
        char hex_torque[5];
        snprintf(hex_torque, 5, "%04X", (uint16_t)torque_current);
        if (strlen(hex_torque) == 3) {
            can_data[5] = strtol(hex_torque, NULL, 16) & 0xFF;
            can_data[4] = (strtol(hex_torque, NULL, 16) >> 8) & 0xFF;
        } else {
            can_data[5] = strtol(hex_torque + 0, NULL, 16) & 0xFF;
            can_data[4] = strtol(hex_torque + 2, NULL, 16) & 0xFF;
        }
        memcpy(data_frame, can_data, sizeof(unsigned char) * 8);
    }
}

void command_speed_control(unsigned char data_frame[8], int32_t speed) {
    unsigned char can_data[8];
    can_data[0] = SPEED_CLOSED_LOOP;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = speed & 0xFF;
    can_data[5] = (speed >> 8) & 0xFF;
    can_data[6] = (speed >> 16) & 0xFF;
    can_data[7] = (speed >> 24) & 0xFF; 
    memcpy(data_frame, &can_data, sizeof(unsigned char));   
}

void command_position_control_1(unsigned char data_frame[8], int32_t position) {
    unsigned char can_data[8];
    can_data[0] = POSITION_CTRL_1;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = (position >> 16) & 0xFF;
    can_data[7] = (position >> 24) & 0xFF; 
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}
void command_position_control_2(unsigned char data_frame[8], float position, int speed) {
    unsigned char can_data[8];
    can_data[0] = POSITION_CTRL_2;
    can_data[1] = 0x00;
    
    if (speed <= 255) {
        can_data[2] = speed & 0xFF;
        can_data[3] = 0x00;
    } else {
        can_data[2] = speed & 0xFF;
        can_data[3] = (speed >> 8) & 0xFF;
    }
    //std::cout << position << std::endl;
    bool is_negative = position < 0;
    position = fabs(position);

    uint32_t new_encoder_data = (uint32_t)(position * 100 * 9);


    if (is_negative) {
        new_encoder_data = ~new_encoder_data + 1;  // Two's complement for negative value
    }

    can_data[4] = new_encoder_data & 0xFF;
    can_data[5] = (new_encoder_data >> 8) & 0xFF;
    can_data[6] = (new_encoder_data >> 16) & 0xFF;
    can_data[7] = (new_encoder_data >> 24) & 0xFF;
     
    memcpy(data_frame, can_data, sizeof(can_data));
}

void command_position_control_3(unsigned char data_frame[8], unsigned int direction, int16_t position) {
    unsigned char can_data[8];   
    can_data[0] = POSITION_CTRL_3;
    can_data[1] = direction;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}
void command_position_control_4(unsigned char data_frame[8], unsigned int direction, int16_t position,int16_t speed) {
    unsigned char can_data[8];
    can_data[0] = POSITION_CTRL_4;
    can_data[1] = direction;
    can_data[2] = speed & 0xFF;
    can_data[3] = (speed >> 8) & 0xFF;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}

void command_torque_control_broadcast(unsigned char data_frame[8], int16_t torque_1, int16_t torque_2, int16_t torque_3, int16_t torque_4) {
    unsigned char can_data[8];
    can_data[0] = torque_1 & 0xFF;
    can_data[1] = (torque_1 >> 8) & 0xFF;
    can_data[2] = torque_2 & 0xFF;
    can_data[3] = (torque_2 >> 8) & 0xFF;
    can_data[4] = torque_3 & 0xFF;
    can_data[5] = (torque_3 >> 8) & 0xFF;
    can_data[6] = torque_4 & 0xFF;
    can_data[7] = (torque_4 >> 8) & 0xFF;
    memcpy(data_frame, &can_data, sizeof(unsigned char));
}


// Placeholder implementation of value_map function
double value_map(int value, int in_min, int in_max, int out_min, int out_max) {
    return (double)(value - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

void message_handler(struct can_frame frame, Motor_Status *motor) {
    unsigned char can_data[8];
    memcpy(can_data, frame.data, sizeof(can_data));
    switch(frame.data[0]){
    
    case 0x30:{
        motor->position_Kp = can_data[2];
        motor->position_Ki = can_data[3];
        motor->speed_Kp = can_data[4];
        motor->speed_Ki = can_data[5];
        motor->torque_Kp = can_data[6];
        motor->torque_Ki = can_data[7];
        break;}

    case 0x32:{
        motor->position_Kp = can_data[2];
        motor->position_Ki = can_data[3];
        motor->speed_Kp = can_data[4];
        motor->speed_Ki = can_data[5];
        motor->torque_Kp = can_data[6];
        motor->torque_Ki = can_data[7];
        break;}

    case 0x33:{
        motor->acceleration = (can_data[7] << 24) | (can_data[6] << 16) | (can_data[5] << 8) | can_data[4];
        break;}
    case 0x90:{
        motor->single_angle_position =  (can_data[3] << 8) | can_data[2];
        motor->encoder_zero_position =  (can_data[5] << 8) | can_data[4];
        motor->encoder_offset =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0x91:{
        motor->encoder_offset =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0x19:{
        motor->encoder_offset =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0x92: {
        uint64_t multi_angle_position = 0;
                
        
        // Extracting encoder data from can_data
        multi_angle_position = ((int64_t)0x00 << 56) |((int64_t)can_data[7] << 48) |((int64_t)can_data[6] << 40) |((int64_t)can_data[5] << 32) | ((int64_t)can_data[4] << 24) | ((int64_t)can_data[3] << 16) | ((int64_t)can_data[2] << 8) | (int64_t)can_data[1];
        
        //std::cerr << multi_angle_position << std::endl;
        //std::cout << multi_angle_position << std::endl;
        
        // Check if the position is negative
        if (can_data[7] & 0x80) { // Most significant bit set
            // Sign extend the negative number
            for (int i = 8; i < 64; ++i) {
                multi_angle_position |= (uint64_t)0xFF << i;
            }
        }

        // Convert to int64_t to handle negative values correctly
        int64_t encoder_dec = (int64_t)multi_angle_position;

        // Divide by 900
        motor->multi_angle_position = (float)encoder_dec / 900.0f;
        //std::cerr << (float)encoder_dec / 900.0f << std::endl;
        break;}
    

    case 0x94:{
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}

    case 0x80:{
        motor->motor_state =  'O'; //OFF
        
        break;}
    case 0x81:{
        motor->motor_state =  'S'; //STOP
        break;}
    case 0x88:{
        motor->motor_state =  'R'; //RUN
        break;}
    case 0x9A:{
        motor->temperature =  can_data[1];
        motor->voltage =  ((int64_t)can_data[4] << 8) | can_data[3];
        motor->error_state =  can_data[7];
        break;}

    case 0x9C:{
        motor->temperature =  can_data[1];
        
        // Extract torque current (int16_t)
        motor->torque_current = ((int16_t)can_data[3] << 8) | can_data[2];
        if (motor->torque_current > 2048)
            motor->torque_current -= 65536;
        motor->torque_current = value_map(motor->torque_current, -2048, 2048, -33, 33);
        
        // Extract speed (int16_t)
        motor->speed = ((int16_t)can_data[5] << 8) | can_data[4];
        if (motor->speed > 32768)
            motor->speed -= 65536;
        motor->speed /= 9.0; // Divide by 9
        
        // Extract single angle position (int16_t)
        motor->single_angle_position = ((int16_t)can_data[7] << 8) | can_data[6];
        if (motor->single_angle_position > 32768)
            motor->single_angle_position -= 65536;
        motor->single_angle_position = value_map(motor->single_angle_position, 0, 65535, 0, 40);
        
        break;}

    case 0x9D:{        
        motor->phase_A_current =  (can_data[3] << 8) | can_data[2];
        motor->phase_B_current =  (can_data[5] << 8) | can_data[4];
        motor->phase_C_current =  (can_data[7] << 8) | can_data[6];
        break;}
    
    case 0xA1:{        
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0xA2:{        
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0xA3:{        
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0xA4:{        
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0xA5:{        
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}
    case 0xA6:{        
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}
    default: {
        printf("Invalid Commands \n");
        break;}

    }

}


void motor_identifier(struct can_frame frame, Leg *leg){
    //printf("Received CAN Message: DLC=%d", frame.data[0]);
    //std::cerr << frame.can_id << std::endl;
    if (frame.can_id == 0x141) {
        message_handler(frame, &leg->shoulder);
        
    } else if (frame.can_id == 0x142) {
        message_handler(frame, &leg->hip);
        
    } else if (frame.can_id == 0x143){
        message_handler(frame, &leg->knee);
        
    }else {
        printf("Fix Motor IDs \n");
    }
}


void readings_corrections(int leg_index, float* torques, float* speeds, float* encoders,
                          float* torques_correction, float* speed_correction, float* angles_correction) {
    // Constants
    const float theta1_offset = -90.0; // Using FL as reference
    const float theta2_offset = 0.0;
    const float theta3_offset = 0.0;
    const float transformation_matrix[4][3] = {{1, 1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, -1, -1}};
    const float shoulder_offset_transform[4] = {1, -1, -1, 1};
    const float chain_ratio = 1.25;
    const float torque_constant = 2.09; // in A/Nm

    // Calculate torque corrections
    torques_correction[0] = torques[0] * torque_constant;
    torques_correction[1] = torques[1] * torque_constant;
    torques_correction[2] = torques[2] * 1.25 * torque_constant;

    // Calculate speed corrections
    speed_correction[0] = speeds[0];
    speed_correction[1] = speeds[1];
    speed_correction[2] = speeds[2] / 1.25;
    
    
    
    // Calculate angle corrections
    float theta1 = (encoders[0] - theta1_offset * shoulder_offset_transform[leg_index]) * transformation_matrix[leg_index][0];
    float theta2 = (encoders[1] - theta2_offset * transformation_matrix[leg_index][1]) * transformation_matrix[leg_index][1];
    float theta3 = (encoders[2] - theta3_offset * transformation_matrix[leg_index][2]) * transformation_matrix[leg_index][2] / chain_ratio;

    angles_correction[0] = theta1;
    angles_correction[1] = theta2;
    angles_correction[2] = theta3;
}



void angle_corrections(int leg_index, float* encoders, float* angles_correction) {
    // Constants
    const double theta1_offset = -90.0; // Using FL as reference
    const double theta2_offset = 0.0;
    const double theta3_offset = 0.0;
    const double transformation_matrix[4][3] = {{1, 1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, -1, -1}};
    const double shoulder_offset_transform[4] = {1, -1, -1, 1};
    const double chain_ratio = 1.25;
  

    // Calculate angle corrections
    double theta1 = (encoders[0] - theta1_offset * shoulder_offset_transform[leg_index]) * transformation_matrix[leg_index][0];
    double theta2 = (encoders[1] - theta2_offset * transformation_matrix[leg_index][1]) * transformation_matrix[leg_index][1];
    double theta3 = (encoders[2] - theta3_offset * transformation_matrix[leg_index][2]) * transformation_matrix[leg_index][2] / chain_ratio;

    angles_correction[0] = theta1;
    angles_correction[1] = theta2;
    angles_correction[2] = theta3;
}

void real_robot_commands_angles(int leg_index, float* positions) {
    // Constants
    const double theta1_offset = -90.0;
    const double theta2_offset = 0.0;
    const double theta3_offset = 0.0;  // Joint, not motor offset
    const double transformation_matrix[4][3] = {{1, 1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, -1, -1}};
    const double shoulder_offset_transform[4] = {1, -1, -1, 1};
    const double chain_ratio = 1.25;

    // Convert joint angles to real robot commands
    positions[0] = positions[0] * transformation_matrix[leg_index][0] + theta1_offset * shoulder_offset_transform[leg_index];
    positions[1]= positions[1] * transformation_matrix[leg_index][1] + theta2_offset * transformation_matrix[leg_index][1];
    positions[2] = (positions[2] * transformation_matrix[leg_index][2] + theta3_offset * transformation_matrix[leg_index][2]) * chain_ratio;
}


void real_robot_commands_torques(int leg_index, float*torques_command) {
    // Constants
    const float chain_ratio = 1.25;
    const float torque_constant = 2.09; // Nm/A

    // Transformation matrix for joint torques
    const float transformation_matrix[4][3] = {{1, 1, 1}, {1, -1, -1}, {-1, 1, 1}, {-1, -1, -1}};

    // Convert joint torques to real robot torque commands
    torques_command[0]= torques_command[0] * transformation_matrix[leg_index][0] / torque_constant;
    torques_command[1] = torques_command[1] * transformation_matrix[leg_index][1] / torque_constant;
    torques_command[2] = torques_command[2] * transformation_matrix[leg_index][2] / torque_constant / chain_ratio;
}

int is_value_in_range(double value, double *value_range) {
    return (value_range[0] <= value && value <= value_range[1]);
}

int Security_Position_Joint(float *desired_positions) {
    double t0 = desired_positions[0];
    double t1 = desired_positions[1];
    double t2 = desired_positions[2];
    
    double workspace[3][2] = {{-115, 115}, {-90, 90}, {-130, 0}};
    
    if (!is_value_in_range(t0, workspace[0])) {
        printf("INVALID Shoulder Joint Position: %.2f\n", t0);
        return 0;
    }
    
    if (!is_value_in_range(t1, workspace[1])) {
        printf("INVALID Hip Joint Position: %.2f\n", t1);
        return 0;
    }
    
    if (!is_value_in_range(t2, workspace[2])) {
        printf("INVALID Knee Joint Position: %.2f\n", t2);
        return 0;
    }
    
    return 1;
}

ControlMode intToMode(int mode) {
    switch (mode) {
        
        case 1:
            return Read_Multi_Turn;
        case 2:
            return Read_Status;
        case 3:
            return Torque_Control;
        case 4:
            return Position_Control;
        
        default:
            return Read_Status; // or any default state you prefer
    }
}
