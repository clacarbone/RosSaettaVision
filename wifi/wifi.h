/***************************************************************************
 * File:   kalman.h
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/
#ifndef _WIFI_H_
#define _WIFI_H_

//================
//   Includes
//================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

//====================
//     Defines
//====================

#define WIFI_PORT 5069

#define MSG_STOP 1
//====================
//     Structures
//====================
struct payload_pkg_trajectory {
	float  v;		//velocità lineare
	float  w;	//velocità angolare
};

typedef unsigned char uint8;

typedef enum{
        SPEED,
        MSG,
	KIN_SPEEDS,
	KIN_PARAMS
}pkg_t;

typedef struct {
        float v;
        float w;
} wifi_vel;

typedef struct {
        int msg;
} wifi_msg;

//==============================
//    Function prototypes
//==============================
int init_wifi(char *ip);

int send_wifi_vel(int sock, float vlin, float omega);

int send_wifi_msg(int sock, int m);
#endif
