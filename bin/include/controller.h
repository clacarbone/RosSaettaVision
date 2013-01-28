#ifndef _CONTROLLER_H
#define	_CONTROLLER_H

//================
//   Includes
//================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "matrix.h"
#include "robot.h"


//====================
//     Defines
//====================

//per scegliere tipo controllo
//#define FORM_CTRL	//controllo formazione rigida
#define SWARM_CTRL	//controllo sciame robot



#ifdef FORM_CTRL
	#define  Kx	0.09	//costanti di prop
	#define  Ky	0.09

	#define	SOGLIA_POS_DES	15

	#define FORM_DIR	"data/controller/"
	#define CTRL_INIT_FILE	"data/controller/ctrl_init.txt"
	#define VIAPOINT_FILE	"data/controller/pos_des.txt"
	#define OBSTACLE_FILE	"data/controller/obstacle.txt"
#endif

#ifdef SWARM_CTRL
	#define SWARM_PARAM_FILE	"data/controller/swarmparam.txt"	//param a b c
	#define V_DES	10.0
	#define W_DES	0.0
#endif


/* Macro per calcolo distanza pt ctrl rob/pt ctrl form
 * TRIANGOLO
 *		   P 		
 * 		  /\
 * 	   	 /  \
 * 	    /____\
 * 	   A	   B
 * 
 */  
//#define YtoY1(x)  ( (1192*(x) >>10 ) ) 

#define V_LIN(angle,ux,uy)	(cos(angle)*ux+sin(angle)*uy)			//v
#define OMEGA(d,angle,ux,uy)	((cos(angle)*uy-sin(angle)*ux)/d)	//w

#define Ux(k,x_d,x,d,angle)	(k*(x_d-x-d*cos(angle)))	//ux
#define Uy(k,y_d,y,d,angle)	(k*(y_d-y-d*sin(angle)))	//uy

#define X_des(x_des_form,l,angle)	(x_des_form+l*cos(angle))
#define Y_des(y_des_form,l,angle)	(y_des_form+l*sin(angle))

//====================
//     Structures
//====================

typedef struct {
	float d;	//distanza del punto controllato del robot dal centro
    float x_ctrl;	//punto controllato robot
    float y_ctrl;
    float l;	//distanza del pt controllato del rob rispetto al pt controllato della formazione        
	float angle;
	float v;
    float w;
    float x_des;	//pos des rob (non usata)
    float y_des;
    } controller_t;

typedef controller_t* controller_p;

float d_rob;	//distanza centro/pt ctrl rob

#ifdef FORM_CTRL
int f_type;
float x_des_form,y_des_form;
float x_form, y_form;
int ultimo_punto=0;
FILE *viapoint_file;
#endif

#ifdef SWARM_CTRL
float a, b, c;
float a_obst, b_obst, c_obst;
#endif


typedef struct
{
  float x;
  float y;
} obstacle;

typedef struct
{
  obstacle *obs;    	//pointer to first element in list
  int numObs;    
} obstacleList;

obstacleList *obsList;

//==============================
//    Function prototypes
//==============================
//void read_param(formation *f);

//void init_formation(formation *f);


//void controller_initialization(controller_p c);
void calcola_input(controller_p c,robot_p r);

void calcola_pt_ctrl(controller_p c, robot_p r);

void leggi_posizione_desiderata();
#endif

