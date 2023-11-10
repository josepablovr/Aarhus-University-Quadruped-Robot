// CAN_BUS.h
#ifndef CAN_BUS_H
#define CAN_BUS

int setupCANSocket(const char *interface);
int sendMessage(int socket, int can_id, const unsigned char data[], int data_length, unsigned int debug_mode);
int receiveMessage(int socket, Motor_Status *status, unsigned int debug_mode);

#endif // CAN_BUS