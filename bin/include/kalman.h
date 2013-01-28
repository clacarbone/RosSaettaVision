/***************************************************************************
 * File:   kalman.h
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/
#ifndef _KALMAN_H_
#define _KALMAN_H_

//================
//   Includes
//================
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "matrix.h"
#include "robot.h"

//====================
//     Defines
//====================

/*
#define MATRIX_A_FILE           "matrixA.txt"
#define MATRIX_W_FILE           "matrixW.txt"
#define MATRIX_H_FILE           "matrixH.txt"
#define MATRIX_V_FILE           "matrixV.txt"
#define MATRIX_Q_PREV_FILE      "matrixQprev.txt"
#define MATRIX_R_FILE           "matrixR.txt"
#define MATRIX_P_FILE           "matrixP.txt"
#define MATRIX_P_PREV_FILE      "matrixPprev.txt"
#define MATRIX_P_MINUS_FILE     "matrixPminus.txt"
#define MATRIX_K_FILE           "matrixK.txt"
*/

#define SAMPLE_TIME 1

#define W_MEAN	0.1
#define V_MEAN	0.1

#define W_COV	0.01	
#define V_COV	0.03

#define PRINT_KALMAN


//#define STATE_INITIAL_CONDITION       "data/state_initial_condition.txt"

//====================
//     Structures
//====================
typedef struct {
    robot_p rob;
    matrix_p est_state;         //a posteriori estimate of the state at step time k
    matrix_p est_state_prev;    //a posteriori estimate of the state at step time k-1
    matrix_p est_state_minus;   //a priori estimate of the state at step time k
    matrix_p evol;

    matrix_p A;                 //A is the Jacobian matrix of partial derivatives of f with respect to x
    matrix_p W;                 //W is the Jacobian matrix of partial derivatives of f with respect to w
    matrix_p H;                 //H is the Jacobian matrix of partial derivatives of h with respect to x
    matrix_p V;                 //V is the Jacobian matrix of partial derivatives of h with respect to v

    matrix_p Q_prev;            //Q is the process noise covariance at step time k-1
    matrix_p R;                 //R is the measurement noise covariance at step time k

    matrix_p P;                 //P is the a posteriori estimate error covariance st step time k
    matrix_p P_prev;            //P_prev is the a posteriori estimate error covariance at step time k-1
    matrix_p P_minus;           //P_minus is the a priori estimate error covariance at step time k
    matrix_p K;                 //K is the Kalman gain at step time k

    float sample_time;
    int initialized;              //1: initialization done
    
    int only_pred;
    
} kalman_t;

typedef kalman_t* kalman_p;




//==============================
//    Function prototypes
//==============================

/*
 * Crea nuova struttura kalman
 */
void new_kalman(kalman_p* k);

/*
 * Inizializza la struttura k
 */
void init_kalman(kalman_p k, robot_p rob, float st);

/*
 * Distrugge la struttura k
 */
void free_kalman(kalman_p k);

/*
 * Imposta le condizioni iniziali per l'algoritmo
 */
void kalman_initialization(kalman_p k);

/*
 * Stima dello stato del sistema
 */
void state_prediction(kalman_p k);

/*
 * Stima della covarianza dell'errore
 */
void cov_prediction(kalman_p k);
/*
 * Fase di predizione
 */
void kalman_prediction(kalman_p k);

/*
 * Correzione del guadagno di kalman
 */
void gain_correction(kalman_p k);
/*
 * Correzione della stima dello stato
 */
void state_correction(kalman_p k);
/*
 * Correzione della covarianza dell'errore
 */
void cov_correction(kalman_p k);

/*
 * Fase di correzione
 */
void kalman_correction(kalman_p k);

/*
 * Aggiornamento della matrice din
 */
void evol_update(kalman_p k);

/*
 * Aggiornamento dei dati
 */
void kalman_update(kalman_p k);

/*
 *  Esegue un passo dell'algoritmo
 */
void kalman_filter(kalman_p k);
#endif
