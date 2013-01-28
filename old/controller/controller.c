#include "controller.h"

/* Legge parametri controllore 
void read_param(formation *f){
	FILE *ff;
	ff=fopen(CTRL_INIT_FILE,"r");
	if(ff==NULL){
		printf("Error on file opening\n");
		exit(-1);
	}
	fscanf(ff,"%d\n%f\n",&(f->type),&d_rob);
	fclose(ff);			
}*/

/*void init_formation(formation *f){
	
}*/


//CONTROLLARE NON USATO
void update_controller(controller_p c, robot_p r){
	//c->rob=r;	
	//calcolo punto controllato robot
	c->x_ctrl=r->state->val[0][0]+d_rob*cosf(r->state->val[2][0]);
	c->y_ctrl=r->state->val[1][0]+d_rob*sinf(r->state->val[2][0]);	
}

void calcola_pt_ctrl(controller_p c, robot_p r){
	//c->rob=r;	
	//calcolo punto controllato robot
	c->x_ctrl=r->state->val[0][0]+d_rob*cosf(r->state->val[2][0]);
	c->y_ctrl=r->state->val[1][0]+d_rob*sinf(r->state->val[2][0]);
	
}

void calcola_input(controller_p c,robot_p r){
	#ifdef FORM_CTRL
	//se x_des_form == HUGE_VALF significa che l'ultimo punto è stato raggiunto quindi fermo i robot
	if(x_des_form==HUGE_VALF){
	c->v=0;
	c->w=0;	
	}
	//altrimenti calcolo nuovo input
	else{	
	float cosfi=cosf(r->state->val[2][0]);	//coseno angolo robot
	float sinfi=sinf(r->state->val[2][0]);	//seno angolo robot
	
	float angle_form=atan2f(y_des_form,x_des_form);
	
	//float x_des,y_des;
	float ux,uy;
	float angle;
	float angle_res=angle_form + c->angle;
	//x_des=x_des_form+c->l*cos(angle_res);
	//y_des=y_des_form+c->l*sin(angle_res);
	printf("L %f ANGLE %f\n",c->l,c->angle);
	c->x_des=x_des_form+c->l*cos(c->angle);//*cos(angle_res);
	c->y_des=y_des_form+c->l*sin(c->angle);//sin(angle_res);
	
	
	//ux=Kx*(x_des-r->state->val[0][0]-c->d*cosfi);
	//uy=Ky*(y_des-r->state->val[1][0]-c->d*sinfi);
	ux=Kx*(c->x_des-c->x_ctrl);
	uy=Ky*(c->y_des-c->y_ctrl);
	c->v=cosfi*ux+sinfi*uy;
	c->w=(1/c->d)*(-sinfi*ux+cosfi*uy);
	
	/*
	if (c->v>15)
		c->v=15;
	if (c->v<-15)
		c->v=-15;
	if (c->w>0.5)
		c->w=0.5;
	if (c->w<-0.5)
		c->w=-0.5;
		*/
}
#endif
	/*
	float cosfi=cosf(r->state->val[2][0]);	//coseno angolo robot
	float sinfi=sinf(r->state->val[2][0]);	//seno angolo robot
	
	float x_des,y_des;
	float ux,uy;
	float angle;
	angle=f.fi_c+M_PI+c->fi_zero;
	x_des=f.x_d+c->l*cos(angle);
	y_des=f.y_d+c->l*sin(angle);
	ux=Kx*(x_des-r->state->val[0][0]-c->d*cosfi);
	uy=Ky*(y_des-r->state->val[1][0]-c->d*sinfi);
	
	c->v=cosfi*ux+sinfi*uy;
	c->w=(1/c->d)*(-sinfi*ux+cosfi*uy);
	*/
}

void leggi_posizione_desiderata(){
	#ifdef FORM_CTRL
	if(ultimo_punto==0){
		int controllo=fscanf(viapoint_file,"%f\t%f\n",&x_des_form,&y_des_form);
		if(controllo==EOF){
			x_des_form=HUGE_VALF;
			y_des_form=HUGE_VALF;
			fclose(viapoint_file);
			ultimo_punto=1;			
		}
	}
	#endif
}

/*
void crea_lista_ostacoli(){
	FILE *fo;
	fo=fopen(OBSTACLE_FILE,"r");
	//conta ostacoli
	int cont=0;
	int ch;
	while(ch=getc(fo)!=EOF)
		if(ch=='\n')
			cont++;
	fclose(fo);
	
	//crea lista	
	*obsList = (obstacleList *)malloc( sizeof( obstacleList ) );
	(*obsList)->numObs=cont;
	(*obsList)->obs = (obstacle *)malloc(sizeof(obsList)*cont);
	
	int i;
	for(i=0;i<cont;i++){
		fscanf(fo,"%f\t%f\n",&((*obsList)->obs[i].x),&((*obsList)->obs[i].x))
	}	
}
*/
