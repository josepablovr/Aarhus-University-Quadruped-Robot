#include "RMD_X8.h"
#include "CAN_BUS.h"

int main() {
    const char *interface = "vcan1";  
    
    int s;

    s = setupCANSocket(interface);
    unsigned char data_frame[8];
    Motor_Status motor1;
    motor1.id = 0x141;
    for (int i = 0; i < 10000; ++i) {  // Loop 1000 times for demonstration
        for (int id = 0x141; id <= 0x141; id +=1) {
            
            
            command_read_encoder(data_frame);
            sendMessage(s, motor1.id, data_frame, 8, 0);

            if (receiveMessage(s, &motor1, 1) == 0) {
                //printf("CAN Message Reception Completed\n");
                continue;
            } else {
                printf("Failed to Receive CAN Messages\n");
            }
            
            
        }}
   





 return 0;
}