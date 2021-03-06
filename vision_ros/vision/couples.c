#include "couples.h"

//==============================================================================
float distanza(float x1, float y1, float x2, float y2){
	return sqrtf(powf((x1-x2),2.0)+powf((y1-y2),2.0));
}
//==============================================================================

void calcola_coord(couple c){
	//printf("TEST ERR x1 %f x2 %f y1 %f y2 %f\n",c.x1,c.x2,c.y1,c.y2);
	//getchar();
	//se le misure sono valide
	if(fabs(c.x1)!=HUGE_VALF && fabs(c.y1)!=HUGE_VALF && fabs(c.x2)!=HUGE_VALF && fabs(c.y2)!=HUGE_VALF){
		c.coord->val[0][0]=(c.x1+c.x2)/2;
		c.coord->val[1][0]=(c.y1+c.y2)/2;	
		c.coord->val[2][0]=(atan2f(c.y1-c.y2,c.x1-c.x2));//*180/M_PI); per conversione in gradi			
		//printf("y1- y2 %f\t x1-x2 %f\n",c.y1-c.y2,c.x1-c.x2);
		//printf("x: %f y: %f\n",c.coord->val[0][0],c.coord->val[1][0]);
		//printf("theta %f\n",c.coord->val[2][0]*180/M_PI);
	//	getchar();
	}
	else{
		c.coord->val[0][0]=HUGE_VALF;
		c.coord->val[1][0]=HUGE_VALF;
		c.coord->val[2][0]=HUGE_VALF;
	}
}

//calcola coordinate per tutte le coppie della lista
void calcola_coord_all(coupleList *list){
		int i;
		for(i=0;i<list->numCpl;i++){
			calcola_coord(list->cpl[i]);
			//print_matrix(list->cpl[i].coord);
		}
	}
//=============================================================================

//Verifica se i marker della coppia sono stati entrambi acquisiti ---> no inf
 int coppia_valida(couple c){
	return (c.coord->val[0][0]!=HUGE_VALF);	//se la prima coord è inf significa che nella coppia c'è almeno un marker con coord +/- inf	
}
//=============================================================================

float distanza_marker(marker m1, marker m2){
	float x1,y1,x2,y2;
	x1=m1.x;
	y1=m1.y;
	x2=m2.x;
	y2=m2.y;
	return sqrtf(powf((x1-x2),2.0)+powf((y1-y2),2.0));
}

int minima_distanza_from_robot(robotList *robList,coupleList *cplList,int index)
{
  int j;
  float dmin=100000;//HUGE_VALF;
  float d;
  float x1,y1;

  int index_min=-1;
  x1=cplList->cpl[index].coord->val[0][0];
  y1=cplList->cpl[index].coord->val[1][0];

  float x2,y2;  
  for(j=0;j<robList->numRob;j++){

    x2=(robList->robots[j]).state->val[0][0];
    y2=(robList->robots[j]).state->val[1][0];
    d=fabs(sqrt(powf((x1-x2),2)+powf((y1-y2),2)));
	if(d<dmin){
		dmin=d;
		index_min=j;
		}
  }
  return index_min;
}

//Restituisce l'indice della coppia a minima distanza dalle punto dato
int minima_distanza(matrix_p pt,coupleList *list){
	int i;
	int index=0;
	float dmin=100000;//HUGE_VALF;
	float x1,y1,x2,y2;
	float d;
	x1=pt->val[0][0];
	y1=pt->val[1][0];
	printf("Point %f %f\n",x1,y1);
	for(i=0;i<cplList->numCpl;i++){				
		x2=cplList->cpl[i].coord->val[0][0];
		y2=cplList->cpl[i].coord->val[1][0];
		printf("Pair %f %f\n",x2,y2);
		d=fabs(sqrt(powf((x1-x2),2)+powf((y1-y2),2)));
		printf("DIST PT/CPL %f\n",d);
		if(d<dmin){
			dmin=d;
			index=i;
		}			
	}
	printf("index %d\n\n",index);
	return index;
}
//==============================================================================
//restituisce l'indice del marker nella lista  mList più vicino al marker m
int distanza_min(marker m, markerList *mList){
		int i;
		int index=0;
		float dmin=HUGE_VALF;
		float d;
		for(i=0;i<mList->numMarker;i++){
			d=distanza_marker(m,mList->markers[i]);
			if(d<dmin){
				dmin=d;
				index=i;
			}			
		}
		return index;
} 


void stampaListe(){
int i;
//PRINTE("\n\n\n\nlista1\t%d\nlista2\t%d\n\n\n",m1List->numMarker,m2List->numMarker);
//stampo liste
printf("MARKER1:\n");
for(i=0;i<m1List->numMarker;i++){
	
	printf("\t%f\t%f\n",m1List->markers[i].x,m1List->markers[i].y);
}

printf("MARKER2:\n");
for(i=0;i<m2List->numMarker;i++){
	printf("\t%f\t%f\n",m2List->markers[i].x,m2List->markers[i].y);
}
}

void stampaCoppie(coupleList *cplList){
	int i;
	
	for(i=0;i<cplList->numCpl;i++){
		printf("COPPIA %d:\t",i);
		printf("x1: %f y1: %f x2: %f y2: %f\n",cplList->cpl[i].x1,cplList->cpl[i].y1,cplList->cpl[i].x2,cplList->cpl[i].y2);
		//printf("\t");
		//print_matrix(cplList->cpl[i].coord);
	}
}


int count_and_init()
{
  int cc;
  int numRed=0;
  int numGreen=0;
  
  for(cc=0;cc<colorBList->numBlob;cc++){
    if(colorBList->blobs[cc].color==1){
      numRed++;
    }
    else
    {
      numGreen++;
    }
  }
  if(m1List != NULL){
	if((m1List)->markers!=NULL){
		free((m1List)->markers);
	}
	free(m1List);
  }
  if(m2List != NULL){
	if((m2List)->markers!=NULL){
		free((m2List)->markers);
	}
	free(m2List);
  }

m1List = (markerList *)malloc( sizeof( markerList ) );
m1List->numMarker=numRed;
m1List->markers = (marker *)malloc(sizeof(marker)*(m1List->numMarker));

m2List = (markerList *)malloc( sizeof( markerList ) );
m2List->numMarker=numGreen;
m2List->markers = (marker *)malloc(sizeof(marker)*(m2List->numMarker));

if(cplList!=NULL){
	if((cplList)->cpl!=NULL){
		free(cplList->cpl);
	}
	free(cplList);
}

int result=GSL_MIN (numRed, numGreen);
cplList = (coupleList *)malloc( sizeof( coupleList ) );
cplList->numCpl=result;
cplList->cpl = (couple *)malloc(sizeof(couple)*(cplList->numCpl));

int i;

for(i=0;i<cplList->numCpl;i++){
		new_matrix(&(cplList->cpl[i].coord));
		init_matrix(cplList->cpl[i].coord,3,1);		
	}

return  result;

}

void populate_lists()
{
  int cc, cc1, cc2;
  int inf1=0; //indice elemento che vale ancora HUGE_VAL
  int inf2=0;	
  float dist;
  float x1,x2,y1,y2;
  int flag;
  
  for(cc=0;cc<colorBList->numBlob;cc++){
        x2=colorBList->blobs[cc].centroidX;	//coordinate nuovo blob
	y2=colorBList->blobs[cc].centroidY;
	flag=0;
//  }
	if(colorBList->blobs[cc].color==1){			
		//verifico se fa parte di un marker già inserito
		// scorro la lista dei marker rossi
		for(cc1=0;cc1<m1List->numMarker;cc1++){
			//per ogni marker nella lista calcolo la distanza dal nuovo
			x1=m1List->markers[cc1].x;	//coordinate marker nella lista
			y1=m1List->markers[cc1].y;
			dist=distanza(x1,y1,x2,y2);
			if(dist<=5) {//se troppo vicini considero come unico blob
				m1List->markers[cc1].x=(x1+x2)/2;
				m1List->markers[cc1].y=(y1+y2)/2;
				flag=1;
			}
		}	

			//altrimenti...
			if(!flag){ 

				if(inf1<m1List->numMarker) {	//se c'è spazio aggiungo nuovo marker
					m1List->markers[inf1].x=x2;
					m1List->markers[inf1].y=y2;
					inf1++;					
				}
				else {	//altrimenti (lista piena) annullo le misure
					for(cc2=0;cc2<m1List->numMarker;cc2++){
						m1List->markers[cc2].x=-HUGE_VAL;
						m1List->markers[cc2].y=-HUGE_VAL;	
						m2List->markers[cc2].x=-HUGE_VAL;
						m2List->markers[cc2].y=-HUGE_VAL;	
					}
					//ed esco dal ciclo
					//continue;
				}										
			}		
		
	}
	else{	//se il blob è azzurro
		//verifico se fa parte di un marker già inserito
		// scorro la lista dei marker azzurri
		for(cc1=0;cc1<m2List->numMarker;cc1++){
			//per ogni marker nella lista calcolo la distanza dal nuovo
			x1=m2List->markers[cc1].x;	//coordinate marker nella lista
			y1=m2List->markers[cc1].y;
			dist=distanza(x1,y1,x2,y2);
			if(dist<=15) {//se troppo vicini considero come unico blob
				m2List->markers[cc1].x=(x1+x2)/2;
				m2List->markers[cc1].y=(y1+y2)/2;
				flag=1;
			}
		}	
			//altrimenti...
		if(!flag){ 
			if(inf2<m2List->numMarker) {	//se c'è spazio aggiungo nuovo marker
				m2List->markers[inf2].x=x2;
				m2List->markers[inf2].y=y2;
				inf2++;	//aggiorno contatore lista marker azzurri
			}
			else {	//altrimenti (lista piena) annullo le misure
				for(cc2=0;cc2<m2List->numMarker;cc2++){
					m1List->markers[cc2].x=-HUGE_VALF;
					m1List->markers[cc2].y=-HUGE_VALF;	
					m2List->markers[cc2].x=-HUGE_VALF;
					m2List->markers[cc2].y=-HUGE_VALF;	
				}
				//ed esco dal ciclo
		
			}				
		}
		
	}

}
}

void create_couples()
{
int disc_robots;
  //Clean the data structures
  
disc_robots=count_and_init();

	//DA QUI NUOVO
int cc, cc1, cc2;
int inf1=0; //indice elemento che vale ancora HUGE_VAL
int inf2=0;	
float dist;
float x1,x2,y1,y2;
int flag;
int d1,d2;
//inizializzo lista marker a HUGE_VAL

for(cc=0;cc<m1List->numMarker;cc++){
	(m1List->markers)[cc].x=HUGE_VALF;
	(m1List->markers)[cc].y=HUGE_VALF;	
				
}

for(cc=0;cc<m2List->numMarker;cc++){
(m2List->markers[cc]).x=HUGE_VALF;
(m2List->markers[cc]).y=HUGE_VALF;
}

//inizializzo lista coppie a HUGE_VAL
for(cc=0;cc<cplList->numCpl;cc++){
	cplList->cpl[cc].x1=HUGE_VALF;
	cplList->cpl[cc].y1=HUGE_VALF;
	cplList->cpl[cc].x2=HUGE_VALF;
	cplList->cpl[cc].y2=HUGE_VALF;
}

//if(m2List->numMarker<m1List->numMarker)
//{
  
populate_lists();



int index_cpl;
int used[cplList->numCpl]; //used[i]=1 se marker2 i-esimo già usato 0 altrimenti
for(cc=0;cc<cplList->numCpl;cc++){
		used[cc]=0;		
}

if(m2List->numMarker<m1List->numMarker)
{
    for(cc=0;cc<cplList->numCpl;cc++){
	cplList->cpl[cc].x2=m2List->markers[cc].x;
	cplList->cpl[cc].y2=m2List->markers[cc].y;
	//printf("TEST 1 x1 %f\n",(*cplList)->cpl[cc].x1);
    }
    
    for(cc=0;cc<m1List->numMarker;cc++){ 
	
	index_cpl=distanza_min(m1List->markers[cc],m2List);//d[i-esimo marker2 e lista marker1]
	//if(distanza_marker((*m2List)->markers[cc],(*m1List)->markers[index_cpl])<15)	{
	if(distanza_marker(m1List->markers[cc],m2List->markers[index_cpl])<25)	{
		if(used[index_cpl]==0){
			cplList->cpl[index_cpl].x1=m1List->markers[cc].x;
			cplList->cpl[index_cpl].y1=m1List->markers[cc].y;
			used[index_cpl]=1;
			
		}
		else{
			d1=distanza(cplList->cpl[index_cpl].x2,cplList->cpl[index_cpl].y2,m2List->markers[cc].x,m2List->markers[cc].y); //nuovo
			d2=distanza(cplList->cpl[index_cpl].x2,cplList->cpl[index_cpl].y2,cplList->cpl[index_cpl].x1,cplList->cpl[index_cpl].y1); //vecchio
			if(d1>d2){
			 	cplList->cpl[index_cpl].x1=m1List->markers[cc].x;
				cplList->cpl[index_cpl].y1=m1List->markers[cc].y;
			}
		}
	}
}	

}

else
  
{
    for(cc=0;cc<cplList->numCpl;cc++){
	cplList->cpl[cc].x1=m1List->markers[cc].x;
	cplList->cpl[cc].y1=m1List->markers[cc].y;
	//printf("TEST 1 x1 %f\n",(*cplList)->cpl[cc].x1);
    }
    
    for(cc=0;cc<m2List->numMarker;cc++){ 
	
	index_cpl=distanza_min(m2List->markers[cc],m1List);//d[i-esimo marker2 e lista marker1]
	//if(distanza_marker((*m2List)->markers[cc],(*m1List)->markers[index_cpl])<15)	{
	if(distanza_marker(m2List->markers[cc],m1List->markers[index_cpl])<25)	{
		if(used[index_cpl]==0){
			cplList->cpl[index_cpl].x2=m2List->markers[cc].x;
			cplList->cpl[index_cpl].y2=m2List->markers[cc].y;
			used[index_cpl]=1;
			
		}
		else{
			d1=distanza(cplList->cpl[index_cpl].x1,cplList->cpl[index_cpl].y1,m1List->markers[cc].x,m1List->markers[cc].y); //nuovo
			d2=distanza(cplList->cpl[index_cpl].x1,cplList->cpl[index_cpl].y1,cplList->cpl[index_cpl].x2,cplList->cpl[index_cpl].y2); //vecchio
			if(d1>d2){
				cplList->cpl[index_cpl].x2=m2List->markers[cc].x;
				cplList->cpl[index_cpl].y2=m2List->markers[cc].y;
			}
		}
	}
}
}

cc=0;
//calcolo coordinate per ogni coppia
calcola_coord_all(cplList);

stampaCoppie(cplList);

/*
//aggiorno lista coppie
//1=ROSSO. Li assegno in sequenza
for(cc=0;cc<(*cplList)->numCpl;cc++){
	(*cplList)->cpl[cc].x1=(*m1List)->markers[cc].x;
	(*cplList)->cpl[cc].y1=(*m1List)->markers[cc].y;
	//printf("TEST 1 x1 %f\n",(*cplList)->cpl[cc].x1);
}
float d1,d2;
//2=CIANO Calcolo minima distanza
for(cc=0;cc<(*cplList)->numCpl;cc++){ 
	
	index_cpl=distanza_min((*m2List)->markers[cc],*m1List);//d[i-esimo marker2 e lista marker1]
	//if(distanza_marker((*m2List)->markers[cc],(*m1List)->markers[index_cpl])<15)	{
	if(distanza_marker((*m2List)->markers[cc],(*m1List)->markers[index_cpl])<25)	{
	if(used[index_cpl]==0){
			(*cplList)->cpl[index_cpl].x2=(*m2List)->markers[cc].x;
			(*cplList)->cpl[index_cpl].y2=(*m2List)->markers[cc].y;
			used[index_cpl]=1;
			
		}
		else{
			d1=distanza((*cplList)->cpl[index_cpl].x1,(*cplList)->cpl[index_cpl].y1,(*m2List)->markers[cc].x,(*m2List)->markers[cc].y); //nuovo
			d2=distanza((*cplList)->cpl[index_cpl].x1,(*cplList)->cpl[index_cpl].y1,(*cplList)->cpl[index_cpl].x2,(*cplList)->cpl[index_cpl].y2); //vecchio
			if(d1>d2){
				(*cplList)->cpl[index_cpl].x2=(*m2List)->markers[cc].x;
				(*cplList)->cpl[index_cpl].y2=(*m2List)->markers[cc].y;
			}
		}
	}
}	

cc=0;

//calcolo coordinate per ogni coppia
calcola_coord_all(cplList);

stampaCoppie(cplList);
*/
}
