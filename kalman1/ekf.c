/*
 *  ekf.c
 *  
 *
 *  Created by andrea on 9/8/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "ekf.h"


void init_ekf(){
	int i;
	float qN[] = {1, 0.1};		// Input NoiSe Covariance
	float vN[] = {1, 1, 0.01};		// System Noise Covariance
	float pN[] = {10, 10, 10};
	float pR[] = {2, 2, 0.1};		// Measurement Noise Covariance
	
	
	
	// Sampling Time
	Tc=1; //0.25;	// 5 Hz
	
	// Input Noise Covariance Matrix
	// Note: this should be made parametric! :-)
	mDiag(&mQ,U_SIZE,qN);
	
#ifdef EKF_DEBUG	
	mPrint(mQ,U_SIZE,U_SIZE);
#endif	
	
	// System Noise Covariance Matrix
	mDiag(&mV,X_SIZE,vN);
#ifdef EKF_DEBUG		
	mPrint(mV,X_SIZE,X_SIZE);	
#endif
	
	mDiag(&mPpred,X_SIZE,pN);	
	mDiag(&mPcorr,X_SIZE,pN);
	
#ifdef EKF_DEBUG		
	mPrint(mPpred,X_SIZE,X_SIZE);	
	mPrint(mPcorr,X_SIZE,X_SIZE);		
#endif	
	
	// Sensor Noise Covariance
	vectR = (float *) malloc(sizeof(float)*X_SIZE);
	for (i=0; i < X_SIZE; i++) 
		vectR[i]=pR[i];
	
	
#ifdef  LOG_EKF_DATA	
	ekf_fd = open(LOG_EKF_DATA_FILENAME, O_CREAT | O_WRONLY | O_TRUNC , S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH);
	printf("Ekf FD: %d\n",ekf_fd);
#endif	
	
	
}

void close_ekf() {
	mFree(mQ,U_SIZE);
	mFree(mV,X_SIZE);	
	mFree(mPpred,X_SIZE);			
	mFree(mPcorr,X_SIZE);
	free(vectR);
	
#ifdef LOG_EKF_DATA
	close(ekf_fd);
#endif
	
}

int isNumber(char c) {
	// A number is composed of
	// 1) all digits
	// 2) '+' '-' '.'
	return (isdigit(c) || (c=='.') || (c=='-') || (c=='+'))?1:0;
}


// To check the difference among two numbers
// Note: the 
int checkEq(float m1, float m2) {
	//	printf("%f\t%f\t-->%d \n",m1,m2,( fabsf(m1-m2) < NUM_TOL)?1:0);
	return ( fabsf(m1-m2) < NUM_TOL)?1:0;
}


void EKF_Prediction(ekf_data xprev, ekf_data *xpred, float *u){
	
	float ctha,stha;
	float **mA;		//[X_SIZE][X_SIZE];
	float **mB;		//[X_SIZE][U_SIZE];
	float **mPprev;	//[X_SIZE][X_SIZE];
	float **mP1;	//[X_SIZE][X_SIZE];
	float **mP2;	//[X_SIZE][X_SIZE];
	
	
	
	printf("V: %f \t W: %f\n",u[0],u[1]);		
	int i,j;
	
	ctha= cos(xprev.th+ u[U_W]*Tc/2);
	stha= sin(xprev.th+ u[U_W]*Tc/2);
	
	// State update
	xpred->x = xprev.x + u[U_V]*Tc*ctha;
	xpred->y = xprev.y + u[U_V]*Tc*stha;
	xpred->th = xprev.th + u[U_W]*Tc;
	if ( xpred->th > 2*M_PI)
		xpred->th -= 2*M_PI;
	
	if ( xpred->th < -2*M_PI)
		xpred->th += 2*M_PI;
	
	// Populate Matrices
	// A = [	1	0	-v*tc*sin(th + w*tc/2)
	//			0	1	v*tc*cos(th + w*tc/2)
	//			0	0			1				]
	mBuild(&mA,X_SIZE,X_SIZE);
	// row 1
	mA[0][0]=1;
	mA[0][1]=0;
	mA[0][2]=-u[U_V]*Tc*stha;
	// row 2
	mA[1][0]=0;
	mA[1][1]=1;
	mA[1][2]=u[U_V]*Tc*ctha;
	// row 3
	mA[2][0]=0;
	mA[2][1]=0;
	mA[2][2]=1;
	
	// B = [	tc*cos(th + w*tc/2)		-v*tc*sin(th + w*tc/2)*tc/2
	//			tc*sin(th + w*tc/2)		v*tc*cos(th + w*tc/2)*tc/2
	//					0						tc					]	
	mBuild(&mB,X_SIZE,U_SIZE);
	// row 1
	mB[0][0]=Tc*ctha;
	mB[0][1]=mA[0][2]*Tc/2;
	// row 2
	mB[1][0]=Tc*stha;
	mB[1][1]=mA[1][2]*Tc/2;
	// row 3
	mB[2][0]=0;
	mB[2][1]=Tc;
	
	
	// Covariance Update (The second term is neglected)
	// Pprev= A*P*A' + B*Q*B' + V
	
	// Allocate matrix
	mBuild(&mPprev,X_SIZE,X_SIZE);
	mBuild(&mP1,X_SIZE,X_SIZE);
	mBuild(&mP2,X_SIZE,X_SIZE);	
	
	// mPprex = xprev.P
	// The pPrev at time k is the mPcorr at time k-1
	//	printf("\nmPcorr k-1\n");
	mCopy(mPprev,X_SIZE,X_SIZE,mPcorr,X_SIZE,X_SIZE);	
	//	mPrint(mPprev,X_SIZE,X_SIZE);
	
	// mP1 = P*A';
	mMultTr2(mP1,X_SIZE,X_SIZE,mPprev,X_SIZE,X_SIZE,mA,X_SIZE,X_SIZE);
	// mP2 = A*P*A'
	mMultTr2(mP2,X_SIZE,X_SIZE,mA,X_SIZE,X_SIZE,mP1,X_SIZE,X_SIZE);
	// mPpred = A*P*A' + V
	mAdd2(mPpred,X_SIZE,X_SIZE,mV,X_SIZE,X_SIZE);
	//	printf("\nmPpred k\n");
	//	mPrint(mPpred,X_SIZE,X_SIZE);
	
	// Free memory
	mFree(mA,X_SIZE);
	mFree(mB,X_SIZE);
	mFree(mPprev,X_SIZE);		
	mFree(mP1,X_SIZE);	
	mFree(mP2,X_SIZE);	
	
}


// Data
// 

void EKF_Correction(ekf_data xpred, ekf_data *xcorr, float *obs ){
	
	
	
	int i;
	int h_line;				// Number of lines for the Jacobian
	float **mJ;
	float **dMeas;			// z-h
	float expMeas[X_SIZE];	// Expected measurements
	

	
	// Compute the expected measurements
	expMeas[0]=xpred.x;
	expMeas[1]=xpred.y;
	expMeas[2]=xpred.th;
	
	
	// Since there is only one webcam there is also only one measurement
	h_line=1;
	
	// Main data Structure for the Jacobian
	mBuild(&mJ, h_line, X_SIZE);
		
	// Compute the Jacobian
	for (i=0; i<h_line; i++) {
		//	Let us build the Jacobian
		mJ[i][0]=1;
		mJ[i][1]=1;
		mJ[i][2]=1;
	}
	
	
	//#ifdef EKF_DEBUG
	printf("\nReal\n");
	printf("x[%f]  y[%f]  th[%f]\n",obs[0], obs[1], obs[2]);
	printf("\n");
	printf("\nExpected\n");
	printf("Xpre --- x: %f\ty: %f\t th: %f\n",xpred.x,xpred.y,xpred.th);	
	//#endif	
	
#ifdef EKF_DEBUG	
	for (i=0; i<h_line; i++){
		printf("Jx: %f\tJy: %f\tJth: %f\n",mJ[i][0],mJ[i][1],mJ[i][2]);
	}
#endif
	
	float **mP1;
	float **mP2;
	float **mP3;	
	float **mK;
	
	// Let us build the matrix 
	// P (n x n)		J (q x n)
	// P1=mPpred*J^T	(n x n) x (n x q) = (n x q)
	mBuild(&mP1,X_SIZE,h_line);
	mMultTr2(mP1, X_SIZE, h_line, mPpred, X_SIZE, X_SIZE, mJ, h_line, X_SIZE);
	//	printf("\nP1=Ppred*J^T\n");	
	//	mPrint(mP1,X_SIZE,h_line);	
	
	// P2= J*P1			(q x n) x (n x q) = (q x q)
	mBuild(&mP2,h_line,h_line);
	mMult(mP2,h_line, h_line, mJ, h_line, X_SIZE, mP1, X_SIZE, h_line);
	//	printf("\nP2=J*P1\n");
	//	mPrint(mP2,h_line,h_line);	
	
	// P2= P2+R			(q x q) + (q x q) = (q x q)
	for (i=0; i<h_line; i++)
		mP2[i][i]+=vectR[i];
	//	printf("\nP2=P2+R\n");	
	//	mPrint(mP2,h_line,h_line);		
	
	
	// P3= inv(P2)		(q x q)
	mBuild(&mP3,h_line,h_line);
	mInv(mP3,h_line,h_line,mP2,h_line,h_line);
	//	printf("\nP3=inv(P2)\n");
	//	mPrint(mP3,h_line,h_line);		
	
	
	// K= P1*P3			(n x q) x (q x q) =	(n x q)
	mBuild(&mK,X_SIZE,h_line);
	mMult(mK,X_SIZE,h_line,mP1, X_SIZE, h_line, mP3, h_line, h_line);	
	//	printf("\nK=P1*P3\n");
	//	mPrint(mK,X_SIZE,h_line);
	
	
	// To build the innovation vector
	mBuild(&dMeas, h_line,1);
	for (i=0; i<h_line; i++) {
		dMeas[i][0]=obs[0]-expMeas[0];	
		dMeas[i][0]=obs[1]-expMeas[1];	
		dMeas[i][0]=obs[2]-expMeas[2];	
	}
	//	mPrint(dMeas,h_line,1);	
	
	float **mP4;
	float **mP5;
	float **mInn;
	// Pk = (I-KJ)P
	
	
	// K*J
	// (n x q) x (q x n) = (n x n)
	mBuild(&mP4,X_SIZE,X_SIZE);	
	mMult(mP4,X_SIZE,X_SIZE,mK,X_SIZE,h_line,mJ,h_line,X_SIZE);
	//	printf("\nmJ\n");
	//	mPrint(mJ,h_line,X_SIZE);
	
	//	printf("\nmP4\n");
	//	mPrint(mP4,X_SIZE,X_SIZE);
	
	
	// (I-K*J)
	mEye(&mP5,X_SIZE);	
	mSub2(mP5,X_SIZE,X_SIZE,mP4,X_SIZE,X_SIZE);
	
	// (I-K*J)*P
	mMult(mPcorr, X_SIZE, X_SIZE, mP5, X_SIZE, X_SIZE, mPpred, X_SIZE, X_SIZE);
	
	//	printf("\nmPcorr\n");
	//	mPrint(mPcorr,X_SIZE,X_SIZE);
	
	// K*vec
	// (n x h_line) x (h_line  x 1)
	mBuild(&mInn, X_SIZE, 1);
	mMult(mInn, X_SIZE, 1, mK, X_SIZE, h_line, dMeas, h_line, 1);
	
	printf("\ndMeas\n");	
	mPrint(dMeas,h_line,1);
	//	int kkk;
	//	for (kkk=0; kkk<h_line; kkk++)
	//		printf("[%d]\t",idJac[kkk]);
	//	printf("\n");
	printf("\nmInn\n");	
	mPrint(mInn,X_SIZE,1);
	
	
	// Xpost
	xcorr->x = xpred.x + mInn[0][0];
	xcorr->y = xpred.y + mInn[1][0];
	xcorr->th = xpred.th + mInn[2][0];
	
	if ( xcorr->th > 2*M_PI)
		xcorr->th -= 2*M_PI;
	
	if ( xcorr->th < -2*M_PI)
		xcorr->th += 2*M_PI;
	
	
	printf("\n");
	printf("Xcorr --- x: %f\ty: %f\t th: %f\n",xcorr->x,xcorr->y,xcorr->th);	
	
	// Free Memory
	mFree(mJ,h_line);
	mFree(mP1,X_SIZE);
	mFree(mP2,h_line);
	mFree(mP3,h_line);
	mFree(mP4,X_SIZE);
	mFree(mP5,X_SIZE);
	mFree(mK,X_SIZE);
	mFree(mInn,X_SIZE);
	mFree(dMeas,h_line);
	
#ifdef LOG_EKF_DATA
	bzero(log_ekf_buf,LOG_EKF_BUF_LEN);
	sprintf(log_ekf_buf,"%f\t%f\t%f\n",xcorr->x,xcorr->y,xcorr->th);
	printf("[%d] %s\n",strlen(log_ekf_buf),log_ekf_buf);
	printf("Written: %d\n",write(ekf_fd, log_ekf_buf, strlen(log_ekf_buf)));
#endif
	
}



