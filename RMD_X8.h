#include <stdio.h>
#include <stdint.h> 

#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H


#define READ_PID_DATA            (0x30)
#define WRITE_PID_TO_RAM         (0x31)
#define WRITE_PID_TO_ROM         (0x32)
#define READ_ACCELERATION        (0x33)
#define WRITE_ACCELERATION       (0x34)
#define READ_ENCODER_DATA         (0x90)
#define WRITE_ENCODER_OFFSET     (0x91)
#define WRITE_CURRENT_POSITION   (0x19)
#define READ_MULTI_TURNS_ANGLE   (0x92)
#define READ_SINGLE_ANGLE        (0x94)
#define READ_MOTOR_STATUS        (0x9A)
#define CLEAR_MOTOR_ERROR_FLAG   (0x9B)
#define READ_MOTOR_STATUS_2      (0x9C)
#define READ_MOTOR_STATUS_3      (0x9D)
#define MOTOR_OFF                (0x80)
#define MOTOR_STOP               (0x81)
#define MOTOR_RUNNING            (0x88)
#define TORQUE_CLOSED_LOOP       (0xA1)
#define SPEED_CLOSED_LOOP        (0xA2)
#define POSITION_CTRL_1          (0xA3)
#define POSITION_CTRL_2          (0xA4)
#define POSITION_CTRL_3          (0xA5)
#define POSITION_CTRL_4          (0xA6)


typedef struct {
    int16_t leg_index;
    int16_t id;

    int8_t  temperature;
    int16_t voltage;
    int32_t acceleration;

    //Flags
    unsigned int error_state;
    unsigned int voltage_status;
    unsigned int temperature_status;  

   
    float torque_current;
    float speed;
    

    //Encoder
    float  multi_angle_position;
    float  single_angle_position;    
    int16_t  encoder_zero_position;
    int16_t  encoder_offset;  

    //Currents
    int16_t phase_A_current;
    int16_t phase_B_current;
    int16_t phase_C_current;

    //PID Data
    int8_t  position_Kp;
    int8_t  position_Ki;
    int8_t  speed_Kp;
    int8_t  speed_Ki;
    int8_t  torque_Kp;
    int8_t  torque_Ki;

    char motor_state;

    float angular_position;
    //Processed data
    
    float joint_angle;
    float joint_speed;
    float joint_torque;

    
} Motor_Status;


typedef struct {
    unsigned int leg_index;    
    Motor_Status shoulder;          // Contador de vueltas
    Motor_Status hip;  // Valor anterior del encoder
    Motor_Status knee;        // Dirección del encoder
} Leg;






typedef enum {
    Read_Multi_Turn,
    Read_Status,
    Torque_Control,
    Position_Control
  
} ControlMode;



void command_read_pid_RAM(unsigned char data_frame[8]);
void command_write_pid_RAM(unsigned char data_frame[8], int8_t  position_Kp, 
int8_t  position_Ki, int8_t  speed_Kp,int8_t  speed_Ki, int8_t  torque_Kp, 
int8_t  torque_Ki);
void command_write_pid_ROM(unsigned char data_frame[8], int8_t  position_Kp, 
int8_t  position_Ki, int8_t  speed_Kp,int8_t  speed_Ki, int8_t  torque_Kp, 
int8_t  torque_Ki);
#endif // CAN_MESSAGES_H
void command_read_acceleration(unsigned char data_frame[8]);
void command_write_acceleration(unsigned char data_frame[8], int32_t  acceleration);
void command_read_encoder(unsigned char data_frame[8]);
void command_write_encoder_offset(unsigned char data_frame[8], int16_t  offset);
void command_write_encoder_zero(unsigned char data_frame[8], int16_t  zero);
void command_read_multi_turn_angle(unsigned char data_frame[8]);
void command_read_single_turn_angle(unsigned char data_frame[8]);
void command_read_motor_status_1(unsigned char data_frame[8]);
void command_clear_error_flag(unsigned char data_frame[8]);
void command_read_motor_status_2(unsigned char data_frame[8]);
void command_read_motor_status_3(unsigned char data_frame[8]);
void command_motor_OFF(unsigned char data_frame[8]);
void command_motor_STOP(unsigned char data_frame[8]);
void command_motor_RUN(unsigned char data_frame[8]);
void command_torque_control(unsigned char data_frame[8], int16_t torque_current);
void command_speed_control(unsigned char data_frame[8], int32_t speed);
void command_position_control_1(unsigned char data_frame[8], int32_t position);
void command_position_control_2(unsigned char data_frame[8], float position, int speed);
void command_position_control_3(unsigned char data_frame[8], unsigned int direction, int16_t position);
void command_position_control_4(unsigned char data_frame[8], unsigned int direction, int16_t position,int16_t speed);
void command_torque_control_broadcast(unsigned char data_frame[8], int16_t torque_1, int16_t torque_2, int16_t torque_3, int16_t torque_4);
void message_handler(struct can_frame frame, Leg *leg);
void message_generator(unsigned char data_frame[8], ControlMode Mode);
void motor_identifier(struct can_frame frame, Leg *leg);
void readings_corrections(int leg_index, float* torques, float* speeds, float* encoders,
                          float* torques_correction, float* speed_correction, float* angles_correction);
void angle_corrections(int leg_index, float* encoders, float* angles_correction);
void real_robot_commands_angles(int leg_index, float* positions);
void real_robot_commands_torques(int leg_index, float*torques_command);
int Security_Position_Joint(float *desired_positions);
ControlMode intToMode(int mode);
