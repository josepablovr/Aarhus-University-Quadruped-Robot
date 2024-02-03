#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <unistd.h> // for sleep function

#include <iostream>
#include <typeinfo>


#include <time.h>
#include <termios.h>
#include "RMD_X8.h"
#include "CAN_BUS.h"



#define MAX_WAIT_TIME_SECONDS 5 // Maximum waiting time in seconds

int receiveMessage2(int socket, Leg *leg, unsigned int debug_mode) {
    struct can_frame frame;

    fd_set read_fds;    // File descriptor set for select()
    struct timeval tv;  // Timeout structure for select()

    // Clear the file descriptor set
    FD_ZERO(&read_fds);
    // Add the socket to the file descriptor set
    FD_SET(socket, &read_fds);
   
    // Set the timeout value
    tv.tv_sec = MAX_WAIT_TIME_SECONDS;
    tv.tv_usec = 0;

    // Wait for the socket to become ready for reading or until timeout
    int ready = select(socket + 1, &read_fds, NULL, NULL, &tv);
    if (ready == -1) {
        perror("Select");
        return -1;
    } else if (ready == 0) {
        printf("Timeout: No CAN message received within %d seconds.\n", MAX_WAIT_TIME_SECONDS);
        return -1; // Or handle the timeout scenario according to your requirements
    }

    // If we reached here, it means there is data available to be read
    // Receive the CAN message
    if (read(socket, &frame, sizeof(struct can_frame)) < 0) {
        perror("Read");
        return -1;
    }
    motor_identifier(frame, leg);

    // Process the received CAN message
    // (Assuming there's code here to handle the received message)
    
    return 0; // Or return appropriate value indicating success
}

int setupCANSocket(const char *interface) {
    int socket_fd; // CAN RAW Socket

    
	struct sockaddr_can addr;
    struct ifreq ifr;
	

    /* open socket */
	if ((socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
		return 1;
	}
    
    strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

    // (error handling omitted for brevity)
    // ...

    // Bind the socket to the specified CAN interface
    // (error handling omitted for brevity)
    // ...

    return socket_fd;
}






int sendMessage(int socket, int can_id, const unsigned char data[], int data_length, unsigned int debug_mode) {
    struct can_frame frame;

    // Prepare the CAN frame
    frame.can_id = can_id;
    frame.can_dlc = data_length;
    memcpy(frame.data, data, data_length);

    // Transmit the CAN frame
    if (write(socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return -1;
    }
    
    if (debug_mode == 1) {
        printf("Sent CAN Message:     BUS=%d, ID=0x%X, DLC=%d, Data=", socket, frame.can_id, frame.can_dlc);
        for (int i = 0; i < frame.can_dlc; i++) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
    }

    return 0;
}



int receiveMessage(int socket, Leg *leg, unsigned int debug_mode) {
    struct can_frame frame;

    // Receive a CAN message
    if (read(socket, &frame, sizeof(struct can_frame)) < 0) {
        perror("Read");
        return -1;
    }
    
    motor_identifier(frame, leg);
    
    if (debug_mode == 1) {
        printf("Received CAN Message: BUS=%d, ID=0x%X, DLC=%d, Data=", socket, frame.can_id, frame.can_dlc);
        
        
        for (int i = 0; i < frame.can_dlc; i++) {
            printf("%02X ", frame.data[i]);
        }
        
        printf("\n");
    }

    return 0;
}

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
