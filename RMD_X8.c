#include "RMD_X8.h"
#include <stdio.h>
#include <stdint.h> 


int countTurns(EncoderCounter *encoder, int current_value) {
   
    if (current_value < 0) {
        current_value = 65536 + current_value;
    }

    if (current_value - encoder->previous_value > 60000) {
        encoder->counter -= 1;
    } else if (current_value - encoder->previous_value < -60000) {
        encoder->counter += 1;
    }

    encoder->previous_value = current_value;
    return encoder->counter;
}


void process_single_turn_angle(EncoderCounter *encoder, Motor_Status *motor) {
    int turns = countTurns(encoder, motor->single_angle_position); 
    motor->position = ((float)motor->single_angle_position+65536*(float)turns)/1638.4;
}



void command_get_motor_status(unsigned char command, unsigned char can_data[8]) {
    can_data[0] = command;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}


void command_read_pid_RAM(unsigned char can_data[8]) {
    can_data[0] = READ_PID_DATA;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_write_pid_RAM(unsigned char can_data[8], int8_t  position_Kp, 
int8_t  position_Ki, int8_t  speed_Kp,int8_t  speed_Ki, int8_t  torque_Kp, 
int8_t  torque_Ki) {

    can_data[0] = WRITE_PID_TO_RAM;
    can_data[1] = 0x00;
    can_data[2] = position_Kp;
    can_data[3] = position_Ki;
    can_data[4] = speed_Kp;
    can_data[5] = speed_Ki;
    can_data[6] = torque_Kp;
    can_data[7] = torque_Ki;

}


void command_write_pid_ROM(unsigned char can_data[8], int8_t  position_Kp, 
int8_t  position_Ki, int8_t  speed_Kp,int8_t  speed_Ki, int8_t  torque_Kp, 
int8_t  torque_Ki) {

    can_data[0] = WRITE_PID_TO_ROM;
    can_data[1] = 0x00;
    can_data[2] = position_Kp;
    can_data[3] = position_Ki;
    can_data[4] = speed_Kp;
    can_data[5] = speed_Ki;
    can_data[6] = torque_Kp;
    can_data[7] = torque_Ki;
}

void command_read_acceleration(unsigned char can_data[8]) {
    can_data[0] = READ_ACCELERATION;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_write_acceleration(unsigned char can_data[8], int32_t  acceleration) {

    can_data[0] = WRITE_ACCELERATION;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = acceleration & 0xFF;
    can_data[5] = (acceleration >> 8) & 0xFF;
    can_data[6] = (acceleration >> 16) & 0xFF;
    can_data[7] = (acceleration >> 24) & 0xFF;
}

void command_read_encoder(unsigned char can_data[8]) {
    can_data[0] = READ_ENCODER_DATA;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_write_encoder_offset(unsigned char can_data[8], int16_t  offset) {
    can_data[0] = WRITE_ENCODER_OFFSET;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = offset & 0xFF;
    can_data[7] = (offset >> 8) & 0xFF;
}

void command_write_encoder_zero(unsigned char can_data[8], int16_t  zero) {
    can_data[0] = WRITE_CURRENT_POSITION;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = zero & 0xFF;
    can_data[7] = (zero >> 8) & 0xFF;
}

void command_read_multi_turn_angle(unsigned char can_data[8]) {
    can_data[0] = READ_MULTI_TURNS_ANGLE;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_read_single_turn_angle(unsigned char can_data[8]) {
    can_data[0] = READ_SINGLE_ANGLE;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_read_motor_status_1(unsigned char can_data[8]) {
    can_data[0] = READ_MOTOR_STATUS;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}
void command_clear_error_flag(unsigned char can_data[8]) {
    can_data[0] = CLEAR_MOTOR_ERROR_FLAG;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_read_motor_status_2(unsigned char can_data[8]) {
    can_data[0] = READ_MOTOR_STATUS_2;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}
void command_read_motor_status_3(unsigned char can_data[8]) {
    can_data[0] = READ_MOTOR_STATUS_3;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}
void command_motor_OFF(unsigned char can_data[8]) {
    can_data[0] = MOTOR_OFF;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_motor_STOP(unsigned char can_data[8]) {
    can_data[0] = MOTOR_STOP;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_motor_RUN(unsigned char can_data[8]) {
    can_data[0] = MOTOR_RUNNING;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = 0x00;
    can_data[5] = 0x00;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}
void command_torque_control(unsigned char can_data[8], int16_t torque_current) {
    can_data[0] = TORQUE_CLOSED_LOOP;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = torque_current & 0xFF;
    can_data[5] = (torque_current >> 8) & 0xFF;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_speed_control(unsigned char can_data[8], int32_t speed) {
    can_data[0] = SPEED_CLOSED_LOOP;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = speed & 0xFF;
    can_data[5] = (speed >> 8) & 0xFF;
    can_data[6] = (speed >> 16) & 0xFF;
    can_data[7] = (speed >> 24) & 0xFF;    
}

void command_position_control_1(unsigned char can_data[8], int32_t position) {
    can_data[0] = POSITION_CTRL_1;
    can_data[1] = 0x00;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = (position >> 16) & 0xFF;
    can_data[7] = (position >> 24) & 0xFF; 
}
void command_position_control_2(unsigned char can_data[8], int32_t position, int16_t speed) {
    can_data[0] = POSITION_CTRL_2;
    can_data[1] = 0x00;
    can_data[2] = speed & 0xFF;
    can_data[3] = (speed >> 8) & 0xFF;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = (position >> 16) & 0xFF;
    can_data[7] = (position >> 24) & 0xFF; 
}
void command_position_control_3(unsigned char can_data[8], unsigned int direction, int16_t position) {
   
    can_data[0] = POSITION_CTRL_3;
    can_data[1] = direction;
    can_data[2] = 0x00;
    can_data[3] = 0x00;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}
void command_position_control_4(unsigned char can_data[8], unsigned int direction, int16_t position,int16_t speed) {
    can_data[0] = POSITION_CTRL_4;
    can_data[1] = direction;
    can_data[2] = speed & 0xFF;
    can_data[3] = (speed >> 8) & 0xFF;
    can_data[4] = position & 0xFF;
    can_data[5] = (position >> 8) & 0xFF;
    can_data[6] = 0x00;
    can_data[7] = 0x00;
}

void command_torque_control_broadcast(unsigned char command, unsigned char can_data[8], int16_t torque_1, int16_t torque_2, int16_t torque_3, int16_t torque_4) {
    can_data[0] = torque_1 & 0xFF;
    can_data[1] = (torque_1 >> 8) & 0xFF;
    can_data[2] = torque_2 & 0xFF;
    can_data[3] = (torque_2 >> 8) & 0xFF;
    can_data[4] = torque_3 & 0xFF;
    can_data[5] = (torque_3 >> 8) & 0xFF;
    can_data[6] = torque_4 & 0xFF;
    can_data[7] = (torque_4 >> 8) & 0xFF;
}



void message_handler(unsigned char can_data[8], Motor_Status *motor) {
    switch(can_data[0]){

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
    case 0x92:{
        motor->multi_angle_position = (0x00 << 56) |(can_data[7] << 48) |(can_data[6] << 40) |(can_data[5] << 32) | (can_data[4] << 24) | (can_data[3] << 16) | (can_data[2] << 8) | can_data[1];
        break;}
    case 0x94:{
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
        break;}

    case 0x80:{
        motor->motor_state =  "OFF";
        break;}
    case 0x81:{
        motor->motor_state =  "STOP";
        break;}
    case 0x88:{
        motor->motor_state =  "RUN";
        break;}
    case 0x9A:{
        motor->temperature =  can_data[1];
        motor->voltage =  (can_data[4] << 8) | can_data[3];
        motor->error_state =  can_data[7];
        break;}

    case 0x9C:{
        motor->temperature =  can_data[1];
        motor->torque_current =  (can_data[3] << 8) | can_data[2];
        motor->speed =  (can_data[5] << 8) | can_data[4];
        motor->single_angle_position =  (can_data[7] << 8) | can_data[6];
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
        printf("Invalid Response \n");
        break;}

    }

}


void motor_identifier(unsigned int id, unsigned char can_data[8], Leg *leg){
    if (id == 0x141) {
        message_Handler(can_data, leg->shoulder);
    } else if (id == 0x142) {
        message_Handler(can_data, leg->hip);
    } else if (id == 0x143){
        message_Handler(can_data, leg->hip);
    }else {
        printf("Fix Motor IDs \n");
    }
}

