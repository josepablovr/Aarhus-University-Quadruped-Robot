// CAN_BUS.h
#ifndef CAN_BUS_H
#define CAN_BUS
#include "can_network_library/RMD_X8.h"

typedef enum {
    IDLE,
    STOP,
    START,
    PAUSE,
    RESTART,
    EXTRA
  
} MainState;

int receiveMessage2(int socket, Leg *leg, unsigned int debug_mode);
int setupCANSocket(const char *interface);
int sendMessage(int socket, int can_id, const unsigned char data[], int data_length, unsigned int debug_mode);
int receiveMessage(int socket, Leg *leg, unsigned int debug_mode);
MainState intToState(int state);
int get_Multi_turn_angle(int s, Leg *leg);
int get_Read_Status(int s, Leg *leg);
int get_Torque_Control(int s, Leg *leg, float torques_command[3]);
int get_Position_Control(int s, Leg *leg, float position_command[3], int speed_pos_ctr[3]);

#endif // CAN_BUS
