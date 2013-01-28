/***************************************************************************
 * File:   kalman.c
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/

#include "kalman.h"



//****************************************************************************
void new_kalman(kalman_p* k){
    //*k = malloc(sizeof(kalman_t));
    //*(k->rob)=malloc(sizeof(robot_t));
    new_matrix(&((*k)->est_state));
    new_matrix(&((*k)->est_state_prev));
    new_matrix(&((*k)->est_state_minus));
    new_matrix(&((*k)->evol));
    new_matrix(&((*k)->A));
    new_matrix(&((*k)->W));
    new_matrix(&((*k)->H));
    new_matrix(&((*k)->V));
    new_matrix(&((*k)->Q_prev));
    new_matrix(&((*k)->R));
    new_matrix(&((*k)->P));
    new_matrix(&((*k)->P_prev));
    new_matrix(&((*k)->P_minus));
    new_matrix(&((*k)->K));
}
//****************************************************************************

void init_kalman(kalman_p k, robot_p rob, float st){
    k->rob=rob;
    init_matrix(k->est_state, STATE_DIM_ROW, STATE_DIM_COL);
    init_matrix(k->est_state_prev, STATE_DIM_ROW, STATE_DIM_COL);
    init_matrix(k->est_state_minus, STATE_DIM_ROW, STATE_DIM_COL);
    init_matrix(k->evol, STATE_DIM_ROW, INPUT_DIM_ROW);
    init_matrix(k->A, STATE_DIM_ROW, STATE_DIM_ROW);
    init_matrix(k->W, STATE_DIM_ROW, STATE_DIM_ROW);
    init_matrix(k->H, MEASURE_DIM_ROW, MEASURE_DIM_ROW);
    init_matrix(k->V, MEASURE_DIM_ROW, MEASURE_DIM_ROW);
    init_matrix(k->Q_prev, STATE_DIM_ROW, STATE_DIM_ROW);
    init_matrix(k->R, MEASURE_DIM_ROW, MEASURE_DIM_ROW);
    init_matrix(k->P, MEASURE_DIM_ROW, MEASURE_DIM_ROW);
    init_matrix(k->P_prev, MEASURE_DIM_ROW, MEASURE_DIM_ROW);
    init_matrix(k->P_minus, STATE_DIM_ROW, STATE_DIM_ROW);
    init_matrix(k->K, MEASURE_DIM_ROW, MEASURE_DIM_ROW);
    k->sample_time=st;
    k->initialized=0;
    k->only_pred=0;
}
//****************************************************************************

void free_kalman(kalman_p k){
    free(k->rob);
    free_matrix(k->est_state);
    free_matrix(k->est_state_prev);
    free_matrix(k->est_state_minus);
    free_matrix(k->evol);
    free_matrix(k->A);
    free_matrix(k->W);
    free_matrix(k->H);
    free_matrix(k->V);
    free_matrix(k->Q_prev);
    free_matrix(k->R);
    free_matrix(k->P);
    free_matrix(k->P_prev);
    free_matrix(k->P_minus);
    free_matrix(k->K);
    free(k);
}
//****************************************************************************
/*VECCHIO DA FILE
void kalman_initialization(kalman_p k){
    //set_initial_state(k->rob);
    copy(k->rob->state,k->est_state);
    copy(k->rob->state,k->est_state_prev);
    copy(k->rob->state,k->est_state_minus);
    read_from_file(k->A,strcat(k->rob->data_dir,MATRIX_A_FILE));
    read_from_file(k->W,strcat(k->rob->data_dir,MATRIX_W_FILE));
    read_from_file(k->H,strcat(k->rob->data_dir,MATRIX_H_FILE));
    read_from_file(k->V,strcat(k->rob->data_dir,MATRIX_V_FILE));
    read_from_file(k->Q_prev,strcat(k->rob->data_dir,MATRIX_Q_PREV_FILE));
    read_from_file(k->P_minus,strcat(k->rob->data_dir,MATRIX_P_MINUS_FILE));
    read_from_file(k->K,strcat(k->rob->data_dir,MATRIX_K_FILE));

   
    //[cos 0; sen 0;0 1]
    k->evol->val[0][0]=cos(k->est_state_prev->val[2][0]);
    k->evol->val[0][1]=0;
    k->evol->val[1][0]=sin(k->est_state_prev->val[2][0]);
    k->evol->val[1][1]=0;
    k->evol->val[2][0]=0;
    k->evol->val[2][1]=1;
}
*/
//NUOVO 
void kalman_initialization(kalman_p k){
    //set_initial_state(k->rob);
    copy(k->rob->state,k->est_state);
    copy(k->rob->state,k->est_state_prev);
    copy(k->rob->state,k->est_state_minus);
    
    //A ---> Jacobiano di f rispetto a x
    k->A->val[0][0]=1;
    k->A->val[0][1]=0;
    k->A->val[0][2]=-k->sample_time*sin(k->rob->state->val[2][0])*v;
    k->A->val[1][0]=0;
    k->A->val[1][1]=1;
    k->A->val[1][2]=k->sample_time*cos(k->rob->state->val[2][0])*v;
    k->A->val[2][0]=0;
    k->A->val[2][1]=0;
    k->A->val[2][2]=1;
    
    
    matrix_p temp;
    new_matrix(&temp);
    init_matrix(temp,3,3);       
    eye(temp);
    //W ---> Jacobiano di f rispetto a w
    //cnst(W_MEAN,k->W);        
    product_scalar_by_matrix(W_MEAN,temp,k->W);
    //H ---> Jacobiano di h rispetto a x
    eye(k->H);    
    //V ---> Jacobiano di h rispetto a v
    //cnst(V_MEAN,k->V); 
    product_scalar_by_matrix(V_MEAN,temp,k->V);
    //Q_prev ---> Cov rumore di processo istante k-1
    //cnst(W_COV,k->Q_prev);    
    product_scalar_by_matrix(W_COV,temp,k->Q_prev);
    //eye(k->W);
    //product_scalar_by_matrix(W_MEAN,temp,k->W);
    //R ---> Cov rumore di misura istante k-1
    //cnst(V_COV,k->R);    
    product_scalar_by_matrix(V_COV,temp,k->R);
    //P_prev ---> Stima cov errore k=0
    eye(k->P_prev);
    		
    // |cos 0|
    // |sen 0|
    // |0   1|
    // funzione f
    k->evol->val[0][0]=cos(k->est_state_prev->val[2][0]);
    k->evol->val[0][1]=0;
    k->evol->val[1][0]=sin(k->est_state_prev->val[2][0]);
    k->evol->val[1][1]=0;
    k->evol->val[2][0]=0;
    k->evol->val[2][1]=1;
    
    //copy(k->rob->state_prev,k->est_state_prev);	//all'inizio la stima dello stato è lo stato stesso
copy(k->rob->state,k->est_state_prev);	//all'inizio la stima dello stato è lo stato stesso    
    k->initialized=1;
}

//****************************************************************************

void state_prediction(kalman_p k){
	matrix_p temp1,temp2;
    new_matrix(&temp1);
    new_matrix(&temp2);
	printf("Debug Attilio\n");
		print_matrix(k->evol);
		print_matrix(k->rob->input_prev);
	        printf("Fine Debug Attilio\n");	
				
    product_rows_by_cols(k->evol,k->rob->input_prev,temp1);
    product_scalar_by_matrix(k->sample_time,temp1,temp2);
    sum(k->est_state_prev,temp2,k->est_state_minus);
    free_matrix(temp1);
    free_matrix(temp2);
}
//****************************************************************************

void cov_prediction(kalman_p k){
	matrix_p trans,temp1,temp2,temp3;
    new_matrix(&trans);
    new_matrix(&temp1);
    new_matrix(&temp2);
    new_matrix(&temp3);
    transpose(k->A,trans);
    product_rows_by_cols(k->A,k->P_prev,temp1);
    product_rows_by_cols(temp1,trans,temp2);

    clear_matrix(trans);
    clear_matrix(temp1);
    transpose(k->W,trans);
    product_rows_by_cols(k->W,k->Q_prev,temp1);
    product_rows_by_cols(temp1,trans,temp3);
    sum(temp2,temp3,k->P_minus);
    free_matrix(trans);
    free_matrix(temp1);
    free_matrix(temp2);
    free_matrix(temp3);    
}
//****************************************************************************

void kalman_prediction(kalman_p k){
	state_prediction(k);
	cov_prediction(k);
    }
//****************************************************************************

void gain_correction(kalman_p k){
    matrix_p inv,transH,trans,temp1,temp2,temp3;
    new_matrix(&inv);
    new_matrix(&transH);
    new_matrix(&trans);
    new_matrix(&temp1);
    new_matrix(&temp2);
    new_matrix(&temp3);
    
    transpose(k->H,transH);
    product_rows_by_cols(k->H,k->P_minus,temp1);
    product_rows_by_cols(temp1,transH,temp2);
    clear_matrix(temp1);
    transpose(k->V,trans);
    product_rows_by_cols(k->V,k->R,temp1);
    product_rows_by_cols(temp1,trans,temp3);
    free_matrix(trans);
    clear_matrix(temp1);
    
    sum(temp2,temp3,temp1);
    free_matrix(temp2);
    free_matrix(temp3);
    printf("prima\n");
   // inverse(temp1,inv);
    
    float det_S = det(temp1);
    
    if(isnan(det_S) || det_S==0)
    {
	init_matrix(inv,3,3);
	zeros(inv);
	k->only_pred++;
    }
    else{
        inverse3x3(temp1,inv);
    }
    printf("dopo\n");
    clear_matrix(temp1);
    product_rows_by_cols(k->P_minus,transH,temp1);
    product_rows_by_cols(temp1,inv,k->K);
    free_matrix(transH);
    free_matrix(temp1);
    free_matrix(inv);    
}
//****************************************************************************

void state_correction(kalman_p k){
	//printf("MISURA: ")
	//se la misura si discosta troppo dalla stima va scartata
	/*
	k->est_state->val[0][0]=k->est_state_minus->val[0][0];
	k->est_state->val[1][0]=k->est_state_minus->val[1][0];
	k->est_state->val[2][0]=k->est_state_minus->val[2][0];
	//*/
	//*
	if(distance(k->est_state_minus,k->rob->measure)>(20+k->only_pred)||fabs(k->rob->measure->val[0][0]==HUGE_VALF)){
	
	//copy(k->est_state_minus,k->est_state);
	k->est_state->val[0][0]=k->est_state_minus->val[0][0];
	k->est_state->val[1][0]=k->est_state_minus->val[1][0];
	k->est_state->val[2][0]=k->est_state_minus->val[2][0];
	
	if(isnan(k->est_state->val[0][0]) || isnan(k->est_state->val[1][0]) || isnan(k->est_state->val[2][0]))
	{
		printf("Errore nella copia dallo stato meno allo stato stimato\n");
		exit(0);
	}
	k->only_pred++;
	}	
	//altrimenti si procede con la correzione
	else{
		matrix_p temp1,temp2;
		new_matrix(&temp1);
		new_matrix(&temp2);
		sub(k->rob->measure,k->est_state_minus,temp1);
		
		product_rows_by_cols(k->K,temp1,temp2);	
				
		temp2->val[0][0]=(k->K->val[0][0]*temp1->val[0][0])+(k->K->val[0][1]*temp1->val[1][0])+(k->K->val[0][2]*temp1->val[2][0]);
		temp2->val[1][0]=(k->K->val[1][0]*temp1->val[0][0])+(k->K->val[1][1]*temp1->val[1][0])+(k->K->val[1][2]*temp1->val[2][0]);
		temp2->val[2][0]=(k->K->val[2][0]*temp1->val[0][0])+(k->K->val[2][1]*temp1->val[1][0])+(k->K->val[2][2]*temp1->val[2][0]);
				
		
		sum(k->est_state_minus,temp2,k->est_state);	
		
		free_matrix(temp1);
		free_matrix(temp2);
		k->only_pred=0;
	}
	//*/
}
//****************************************************************************

void cov_correction(kalman_p k){
    /*vecchio ---> P_k=(I-K_k*H_k)P_k_minus
    matrix_p e,temp1,temp2;
    new_matrix(&e);
    new_matrix(&temp1);
    new_matrix(&temp2);

    product_rows_by_cols(k->K,k->H,temp1);
    init_matrix(e,temp1->row,temp1->col);
    eye(e);
    sub(e,temp1,temp2);
    product_rows_by_cols(temp2,k->P_minus,k->P);
    free_matrix(e);
    free_matrix(temp1);
    free_matrix(temp2);
	*/
	/*nuovo con forma di Joseph ---> P_k=(I-K_k*H_k)P_k_minus(I-K_k*H_k)'+K_k*R*K_k' */
	matrix_p temp0,temp1,temp2, temp3;
    new_matrix(&temp0);
    new_matrix(&temp1);
    new_matrix(&temp2);
    new_matrix(&temp3);
    
    product_rows_by_cols(k->K,k->H,temp1);	//temp1 = K*H
    init_matrix(temp0,temp1->row,temp1->col);	//temp0 = matrice identità
    eye(temp0);
    sub(temp0,temp1,temp2);		//temp2 = I-K*H
    clear_matrix(temp0);
    clear_matrix(temp1);
    product_rows_by_cols(temp2,k->P_minus,temp0);	//temp0 = (I-K*H)*P_minus
    transpose(temp2,temp1);	//temp1 = (I-K*H)'
    clear_matrix(temp2);
    product_rows_by_cols(temp0,temp1,temp2);	//temp2=(I-K*H)*P_minus*(I-K*H)'
    clear_matrix(temp0);
    clear_matrix(temp1);
    product_rows_by_cols(k->K,k->R,temp0);	//temp0=K*R
    transpose(k->K,temp1);
    product_rows_by_cols(temp0,temp1,temp3); //temp3=K*R*K'
    sum(temp2,temp3,k->P);		//P -->forma di joseph    
    free_matrix(temp0);
    free_matrix(temp1);
    free_matrix(temp2);
    free_matrix(temp3);
}
//****************************************************************************

void kalman_correction(kalman_p k){
	gain_correction(k);
	
	state_correction(k);
	
	cov_correction(k);   	
}
//****************************************************************************

void evol_update(kalman_p k){
    k->evol->val[0][0]=cos(k->est_state_prev->val[2][0]);
    k->evol->val[1][0]=sin(k->est_state_prev->val[2][0]);
}
//****************************************************************************

void kalman_update(kalman_p k){
	//getchar();
    
    copy(k->est_state,k->rob->state);	//aggiorno lo stato del robot con quello stimato
    copy(k->P,k->P_prev);	//P si può togliere sostituendo P a P_prev in cov_correction
    copy(k->est_state,k->est_state_prev);	//est_state si può togliere e usare solo est_state_prev
    evol_update(k);    //evol è già aggiornata per il passo successivo    
}
//****************************************************************************

void kalman_filter(kalman_p k){
	kalman_prediction(k);
	
    kalman_correction(k);
   
    kalman_update(k);    
   
}
//****************************************************************************
