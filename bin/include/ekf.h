/*
 *  ekf.h
 *  
 *
 *  Created by andrea on 9/8/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */


#ifndef EKF_H
#define EKF_H

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>		// open
#include <sys/stat.h>	// stat
#include <ctype.h>		// isdigit
#include <math.h>		// fabs

#include "matrix_a.h"		// Matrix floating operations


#define NUM_TOL		1E-10	// 10^(-10)

// State
#define X_SIZE	3
#define X
#define Y
#define TH

// Input
#define U_SIZE	2
#define U_V	0
#define U_W	1

#define sign(x) (x > 0) - (x < 0)

// Ekf Types

typedef struct odo_ odo;
struct odo_ {
	float x;
	float y;
	float th;	
};


typedef struct ekf_data_ ekf_data;
struct ekf_data_ {
	float x;
	float y;
	float th;
};


float Tc;					// Sampling time
float **mPpred; // [X_SIZE][X_SIZE];	// Prediction Covariance Matrix
float **mPcorr;	// [X_SIZE][X_SIZE];	// Correction Covariance Matrix
float **mQ;		// [U_SIZE][U_SIZE];	// Input Noise Covariance Matrix
float **mV;		// [X_SIZE][X_SIZE];	// System Noise Covariance Matrix
float *vectR;	// [X_SIZE];			// Measurement Nose Covariance


#ifdef LOG_EKF_DATA
#include <string.h> 	// required for bzero
#define LOG_EKF_DATA_FILENAME "log_ekf_data.txt"
int ekf_fd;
#define LOG_EKF_BUF_LEN	256
char log_ekf_buf[LOG_EKF_BUF_LEN];
#endif

// Ekf function
void init_ekf();
void close_ekf();

// Xprev, Xpred, u
void EKF_Prediction(ekf_data, ekf_data *, float *);
// Xpred, Xcorr, obs
void EKF_Correction(ekf_data, ekf_data *, float *);


#endif

