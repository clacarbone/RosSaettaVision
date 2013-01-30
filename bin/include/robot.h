/***************************************************************************
 * File:   robot.h
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/

#ifndef _ROBOT_H
#define	_ROBOT_H

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


#include "matrix.h"


//====================
//     Defines
//====================
#define STATE_DIM_ROW   3
#define STATE_DIM_COL   1
#define MEASURE_DIM_ROW 3
#define MEASURE_DIM_COL 1
#define INPUT_DIM_ROW   2
#define INPUT_DIM_COL   1

//#define INITIAL_STATE_FILE  "robot_initial_condition.txt"


//====================
//     Structures
//====================

typedef struct {
    int robot_id;
    char ip[15];
    //char *ip;
    matrix_p state;             //[x_k y_k th_k]' robot state at step time k
    matrix_p state_prev;        //[x_k-1 y_k-1 th_k-1]' robot state at step time k-1
    matrix_p measure;           //[x_k y_k th_k]' measurement at step time k
    matrix_p input;             //[v_k w_k]' control input at step time k
    matrix_p input_prev;        //[v_k-1 w_k-1]' control input at step time k-1
    //char* data_dir;
    char data_dir[30];
} robot_t;

typedef robot_t* robot_p;

//controlli
float v,w; //vel lin e vel ang

//==============================
//    Function prototypes
//==============================
/*
 * Crea un nuovo robot
 */
void new_robot(robot_p* rob);

/*
 * Inizializza il robot rob assegnandogli come ID il valore rob_id
 */
void init_robot(robot_p rob, int rob_id);

/*
 * Distrugge il robot rob
 */
void free_robot(robot_p rob);

/*
 * Aggiorna il vettore delle misure del robot rob
 */
void update_measure(robot_p rob, matrix_p new_m);

/*
 * Aggiorna il vettore di stato del robot rob
 */
void update_state(robot_p rob,matrix_p new_s);

/*
 * Assegna lo stato iniziale al robot
 */
//void set_initial_state(robot_p rob);
//void set_initial_state(robot_p rob, float x, float y, float th);
void set_initial_state(robot_p rob, matrix_p coord);
#endif
