// CAN_BUS.h
#ifndef CAN_BUS_H
#define CAN_BUS


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

#endif // CAN_BUS
