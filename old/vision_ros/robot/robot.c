/***************************************************************************
 * File:   robot.c
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/
#include "robot.h"
//****************************************************************************

void new_robot(robot_p* rob){
    //*rob=malloc(sizeof(robot_t));
    new_matrix(&((*rob)->state));
    new_matrix(&((*rob)->state_prev));
    new_matrix(&((*rob)->measure));
    new_matrix(&((*rob)->input));
    new_matrix(&((*rob)->input_prev));
    //(*rob)->ip=(char*)malloc(sizeof(char)*15);  
}
//****************************************************************************

void init_robot(robot_p rob, int rob_id){
    rob->robot_id=rob_id;
    //per ogni robot crea una directory
    sprintf(rob->data_dir,"data/robot%d/",rob->robot_id);
    mkdir(rob->data_dir,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    init_matrix(rob->state, STATE_DIM_ROW,STATE_DIM_COL);
    init_matrix(rob->state_prev, STATE_DIM_ROW,STATE_DIM_COL);
    init_matrix(rob->measure, MEASURE_DIM_ROW,MEASURE_DIM_COL);
    init_matrix(rob->input, INPUT_DIM_ROW, INPUT_DIM_COL);
    init_matrix(rob->input_prev, INPUT_DIM_ROW, INPUT_DIM_COL);    
}
//****************************************************************************

void free_robot(robot_p rob){
    free(rob->data_dir);
    free_matrix(rob->state);
    free_matrix(rob->state_prev);
    free_matrix(rob->measure);
    free_matrix(rob->input);
    free_matrix(rob->input_prev);    
    free(rob);
}
//****************************************************************************

void update_measure(robot_p rob, matrix_p new_m){
    int i,j;
    if(!new_m){
        printf("Error: measurement not avalaible");
        return;
    }
    if(!sameSize(rob->measure,new_m)){
        printf("Error: dimension must agree");
        return;
    }    
    for(i=0;i<MEASURE_DIM_ROW;i++)
        for(j=0;j<MEASURE_DIM_COL;j++)
            rob->measure->val[i][j]=new_m->val[i][j];
}
//****************************************************************************

void update_state(robot_p rob,matrix_p new_s){
    int i,j;
    if(!new_s){
        printf("Error: measure not avalaible");
        return;
    }
    if(!sameSize(rob->state,new_s)){
        printf("Error: dimension must agree");
        return;
    }
    for(i=0;i<MEASURE_DIM_ROW;i++)
        for(j=0;j<MEASURE_DIM_COL;j++){
            rob->state_prev->val[i][j]=rob->state->val[i][j];
            rob->state->val[i][j]=new_s->val[i][j];
        }
}
//****************************************************************************
//nuovo (da webcam)
void set_initial_state(robot_p rob, matrix_p coord){
		//copy(coord,rob->state_prev);
		copy(coord,rob->state);
}
/*void set_initial_state(robot_p rob, float x, float y, float th){
    
    rob->state_prev->val[0][0]=x;
    rob->state_prev->val[1][0]=y;
    rob->state_prev->val[2][0]=th;
    
    rob->state->val[0][0]=x;
    rob->state->val[1][0]=y;
    rob->state->val[2][0]=th;
    
}*/
/*vecchio (da file)
void set_initial_state(robot_p rob){
    FILE *init_state;
    init_state=fopen(strcat(rob->data_dir,INITIAL_STATE_FILE),"r");
    if(init_state==NULL) {
	printf("Error on file opening\n");
    }
    int i,j,matrix_full;
    matrix_full=0;
    i=j=0;
    while (fscanf(init_state, "%f\n",rob->state->val[i][j]) != EOF) {
        rob->state_prev->val[i][j];
        rob->measure->val[i][j];
        if (j == STATE_DIM_COL - 1) {
            j = 0;
            i++;
            if (i == STATE_DIM_ROW)
                matrix_full = 1;
        } else
            j++;
    }
}
*/
//****************************************************************************
