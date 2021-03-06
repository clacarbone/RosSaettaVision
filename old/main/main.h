#ifndef _LANDER_H_
#define _LANDER_H_

//Libraries inclusion
//===================
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>

//#include <time.h>

#include "globals.h"
#include "v4l2_capture.h"
//#include "coordinates.h"
#include "couples.h"
#include "robot.h"
#include "kalman.h"
#include "controller.h"
#include "wifi.h"
#include "ekf.h"
#include "kinectPackages.h"
//===================
#define TRUE	1
#define NR	5		//numero di robot
#define MYPORT 42000
//Structures
//===================


robotList *robList;

typedef struct {
	kalman_p kalmans;
	int numKalman;
} kalmanList;

kalmanList *kList;

typedef struct {
	controller_p controllers;
	int numController;
} controllerList;

controllerList *ctrlList;



//===================


//Constant values define
#define 			WEBCAM
#define				WIFI
#define				CON_ROBOT
#define 			LOG		//per salvare dati su file
//#define				PRINT		//per abilitare stampe a schermo
//#define TIME_CTRL	//per controllo tempo esecuzione

#define KALMAN_FILE	"data/kalman.txt"	//matrici kalman ad ogni iter
#define MISURE_KALMAN_FILE	"data/misurekalman.txt"	//per confronto tra misura e stima
#define STATO_KALMAN_FILE	"data/statokalman.txt"	//per confronto tra misura e stima
#define IP_FILE	"data/main/listaIp.txt"
#define ROBOT_IP_FILE	"../../main/data/robotIp.txt"
#define LOG_FILE1	"data/logfile1.txt"
#define LOG_FILE2	"data/logfile2.txt"
#define TIME_FILE	"data/timefile.txt"
#define PASSINO_FILE	"data/passino.txt"


#define IP_DISP	10	//num ip disponibili ---> togliere


//distanza tra i punti P1=(x1,y1) e P2=(x2,y2)
#define DISTANZA(x1,y1,x2,y2)	(sqrt(pow((x1-x2),2)+pow((y1-y2),2)))
//conversione gradi/radianti
#define DGR_TO_RAD(x)	(x*M_PI/180)
//===================

//Variables needed for a normal execution of the main thread
char		*vision_map;
char		*vision_map_sphere = "data/vision/Mappa90.txt\0";
char		*vision_mode;
char		*vision_mode_sphere = "sphere_\0";

FILE *flog1, *flog2;
FILE *ftime;
int c_t=0;

//Kalman structure
/*ekf_data x_pred[3];
ekf_data x_post[3];
*/


//OBSTACLES. They are represented by a dynamic array of array of float. 
//example: obstacles[0][2] is the second coordinate of the first obstacle. In our scenario, the second coordinate is on the y-axis.
float **obstacles;
//===================

//Variables needed in order to instanciate a new thread
pthread_t	thread_main;
pthread_t	thread_kinect;
//===================
int *sock;

/**Termination handler is the function able to catch CTRL_C SIGNAL SIGINT (SIGTERM)
*@param[in] signum Integer representing signal
*/
void 		termination_handler(int signum);
#endif
