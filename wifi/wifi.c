/***************************************************************************
 * File:   kalman.c
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/

#include "wifi.h"


int init_wifi(char *ip) {
	int sock;
	struct sockaddr_in server_addr;
	struct hostent *host;
	
	host= (struct hostent *) gethostbyname((char *) ip);
	sock = socket(AF_INET, SOCK_STREAM, 0);

	if (sock == -1)	{
		perror("socket");
		exit(1);
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(WIFI_PORT);
	server_addr.sin_addr = *((struct in_addr *)host->h_addr);
	bzero(&(server_addr.sin_zero),8);

    if (connect(sock, (struct sockaddr *)&server_addr,sizeof(struct sockaddr)) == -1) {
        perror("Connect");
        exit(1);
    }
    return sock;
}

int send_wifi_vel(int sock, float vlin, float omega){
	long int counter=0;
	int sent;
	struct js_event js;
	//struct payload_pkg_trajectory pay;	// payload
	
	uint8 *payload=(uint8*)malloc(sizeof(uint8)*2);
	uint8 size;
	wifi_vel velocities;
    payload[0]=(uint8)SPEED;
    payload[1]=(uint8)(sizeof(wifi_vel));
    size=sizeof(uint8)*2+sizeof(wifi_vel);
    payload=(uint8*)realloc(payload,size);
	velocities.v=vlin;
	velocities.w=omega;
	
	memcpy(payload+2,&velocities,sizeof(wifi_vel));
	
	send(sock, payload, size, 0);
	
    free(payload);
    //close(sock);


	//pay.v=vlin;
	//pay.w=omega;

	//sent=send(sock, (void*)&pay, sizeof(struct payload_pkg_trajectory), 0);
	return 0;
}

int send_wifi_msg(int sock, int m){	

	long int counter=0;
	int sent;
	struct js_event js;
	//struct payload_pkg_trajectory pay;	// payload
	
	uint8 *payload=(uint8*)malloc(sizeof(uint8)*2);
	uint8 size;
	wifi_msg msg;
    payload[0]=(uint8)MSG;
    payload[1]=(uint8)(sizeof(wifi_msg));
    size=sizeof(uint8)*2+sizeof(wifi_msg);
    payload=(uint8*)realloc(payload,size);
	msg.msg=m;
	memcpy(payload+2,&msg,sizeof(wifi_msg));
	send(sock, payload, size, 0);
    free(payload);
    //close(sock);


	//pay.v=vlin;
	//pay.w=omega;

	//sent=send(sock, (void*)&pay, sizeof(struct payload_pkg_trajectory), 0);
	return 0;
}
