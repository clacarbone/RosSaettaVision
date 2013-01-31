//==========================================================================
//
//  Project:        guarDIAn: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Distance computation from pixel indexes
//
//  Description:    A set of function to manage pixel to XY coordinates 
//					correspondence.
//
//  Author:         Enrico Di Lello
//
//  
//--------------------------------------------------------------------------
//  FoxVision: Vision system for mobile robots
//  Copyright (c) 2009 Enrico Di Lello
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307  USA
//  or obtain a copy from the GNU website at http://www.gnu.org/
//
//=============================================================================
#include "globals.h"
#include "coordinates.h"

//==============================================================================

void globals_init(){
	
	FILE *ff;
    ff=fopen(INIT_FILE,"r");
    
    if(ff==NULL) 
        printf("Error on file opening\n");
    fscanf(ff,"%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\t%d\n%d", &SAVE_FRAMES, &SAVE_NORMALIZED, &SAVE_BINARY, &SAVE_COLOR_IMG, &SAVE_CENTROIDS, &n_save_frame, &DEFAULT_HOR_RESOLUTION, &DEFAULT_VERT_RESOLUTION, &DEFAULT_FPS, &LOGGING,&BLOB_MIN_SIZE,&BLOB_MAX_SIZE,&numRobots);
    fclose(ff);

}
//==============================================================================

int InitPixelMap(char *map_file) {
	printf("Initializaing Map module ... \n");
		frame_height=DEFAULT_VERT_RESOLUTION;
		frame_width=DEFAULT_HOR_RESOLUTION;
	CreatePixelMap(&pixelMap,frame_height,frame_width); 	
	FillPixelMap(map_file, pixelMap);	
	return 0;
}

//==============================================================================

int EvaluateCoordinates() {
	//GetColorBlob(&colorBList,&colorRList, &colorMList, compList, pixelMap,cam_log_file);
	GetColorBlob(&colorBList, compList, pixelMap,cam_log_file);
	create_couples();
	return 0;
} 


//==============================================================================


int CreatePixelMap(PixelMap **pixelMap, int rows, int cols){
	 
    int i;
	*pixelMap = (PixelMap *)malloc( sizeof( PixelMap ) );
	
	(*pixelMap)->rows = rows;
	(*pixelMap)->cols = cols;
	(*pixelMap)->x_coord = (float*)malloc((rows*cols)*sizeof(float));
	(*pixelMap)->y_coord = (float*)malloc((rows*cols)*sizeof(float));
	
	for(i=0;i<(*pixelMap)->rows*(*pixelMap)->cols;i++){
		(*pixelMap)->x_coord[i] = 0;
		(*pixelMap)->y_coord[i] = 0;
	}
	return 0;
}


//==============================================================================


int FillPixelMap(char *map_file, PixelMap *pixelMap) {
	
	FILE *map;

	map = fopen(map_file,"r");
	char line[MAX_LINE_LEN];
	int i;
	float indiceRiga;
	float indiceColonna;
	float x_coord;
	float y_coord;
	
	if (map == NULL)
	{
		printf("InitPixelMap: failed to open file\n");
		printf("%s\n",map_file);
	   // getchar();
		return -1;
	}
	printf("%s\n",map_file); //GIO
	//Read data from file
	for(i=0; i<pixelMap->rows*pixelMap->cols; i++){
		
		//Read one line at a time
		fgets(line,MAX_LINE_LEN,map);
		
		if (line == NULL)
		{
			printf("InitPixelMap: failed to read from file\n");
			fclose(map);
			//getchar();
			return -1;
		}
		
		else {
			sscanf(line,"%f\t%f\t%f\t%f",&indiceRiga,&indiceColonna,&x_coord,&y_coord);
			pixelMap->x_coord[i] = x_coord;
			pixelMap->y_coord[i] = y_coord;
		}
	}
	
	fclose(map);
	
	return 0;
}

//==============================================================================


int GetCoordinates(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit, float *packet_data) {

	struct timeval time;
	long timestamp;
	int i,centroidIndex;
	int imageRows,imageCols;

	imageRows = pixelMap->rows;
	imageCols = pixelMap->cols;
	
	//printf("Col: %d\tRow: %d\n",imageCols,imageRows);
	ComponentsListElement*  compListElement;
	Component* comp;
	
	compListElement = compList->head;
	
	//debug
	//PrintPixelMap(pixelMap);
		
	gettimeofday( &time, NULL );
	timestamp =  ((time.tv_sec) * 1000000 + time.tv_usec);
		
	//for each component
	
	// printf("component count: %d\n",compList->count); // Andrea
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		// Get centroid distance only if the blob is big enough
		
		if ( comp->pixelList->count>BLOB_MIN_SIZE) {
	
			centroidIndex = comp->centroidRow*imageCols + comp->centroidCol;
			//QUI//printf("Index: %d\n",centroidIndex);
			comp->centroidX = pixelMap->x_coord[centroidIndex];
			comp->centroidY = pixelMap->y_coord[centroidIndex];
			
			//printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex);
			//printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f , %f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex,comp->centroid_distance,comp->centroid_angle);


			
		// } 	// We log only if a significan blob has been found [Andrea]
		
		// If required, save coordinates in text file
		
			if (LOGGING){
			// Andrea 			
				LogCoordData(comp->centroidX,comp->centroidY,timestamp,compList->count );
				//fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d\n",comp->centroidX,comp->centroidY,timestamp,compList->count );
		
			}		
		
			// If required, prepare data for Xbee transmission
		
			if (transmit){
				packet_data[i*2] = comp->centroidX;
				packet_data[i*2+1] = comp->centroidY;
			}
		
		}	
		compListElement = compListElement->next;
		
		//Andrea		
	}
	
	
	// Andrea
	if (compList->count>0)
		ClearComponentsList(compList);
	
	return 0;
}

//==============================================================================


int GetCoordinatesEvo(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit, float *packet_data) {
	
	struct timeval time;
	long timestamp;
	int i,centroidIndex;
	int imageRows,imageCols;

	imageRows = pixelMap->rows;
	imageCols = pixelMap->cols;
	
	//printf("Col: %d\tRow: %d\n",imageCols,imageRows);
	ComponentsListElement*  compListElement;
	Component* comp;
	
	compListElement = compList->head;
	
	//debug
	//PrintPixelMap(pixelMap);
		
	gettimeofday( &time, NULL );
	timestamp =  ((time.tv_sec) * 1000000 + time.tv_usec);
		
	//for each component
	
	// printf("component count: %d\n",compList->count); // Andrea
	for(i=0;i<compList->count;i++)
	{
		comp = &compListElement->component;
		
		// Get centroid distance only if the blob is big enough
		
		if ( (comp->pixelList->count>BLOB_MIN_SIZE) & (comp->pixelList->count<BLOB_MAX_SIZE) ) {
	
			centroidIndex = comp->centroidRow*imageCols + comp->centroidCol;
			//QUI//printf("Index: %d\n",centroidIndex);
			
			comp->centroidX = pixelMap->x_coord[centroidIndex]/10;
			comp->centroidY = pixelMap->y_coord[centroidIndex]/10;
			comp->centroid_distance=GetDistCentroidBlob(comp->centroidX,comp->centroidY);
			comp->centroid_angle=GetAngleCentroidBlob(comp->centroidX,comp->centroidY);

			//printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex);
			//printf("component %d centroid with color %d [X,Y] = [%f ,%f] \n",i,comp->color,comp->centroidX,comp->centroidY);
			//printf("[X,%f ,Y,%f , Distance,%f , Angle,%f] \n",comp->centroidX,comp->centroidY,comp->centroid_distance,comp->centroid_angle);
			
//			printf("Component %d, color %d [X,%f ,Y,%f , Distance,%f , Angle,%f] \n",i,comp->color,comp->centroidX,comp->centroidY,comp->centroid_distance,comp->centroid_angle);

			
		// } 	// We log only if a significan blob has been found [Andrea]
		
		// If required, save coordinates in text file
		
			if (LOGGING){
			// Andrea 			
				LogCoordDataEvo(comp->centroidX,comp->centroidY,timestamp,compList->count,comp->color);
				//fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d\n",comp->centroidX,comp->centroidY,timestamp,compList->count );
		
			}		
		
			// If required, prepare data for Xbee transmission
		
			if (transmit){
				packet_data[i*2] = comp->centroidX;
				packet_data[i*2+1] = comp->centroidY;
			}
		
		}	
		compListElement = compListElement->next;
		
		//Andrea		
	}

	// Andrea 
	
	if (compList->count>0)
		ClearComponentsList(compList);
	
	return 0;
}



//==============================================================================
/*void printRobot(int i,robotList *robList){
PRINTE("ROBOT %d: marker1: [%f,%f], marker2: [%f,%f], robot: [%f,%f,%f,%f]\n",robList->robots[i].id,robList->robots[i].xm1,robList->robots[i].ym1,robList->robots[i].xm2,robList->robots[i].ym2,robList->robots[i].x,robList->robots[i].y,robList->robots[i].distance,robList->robots[i].orientation);
if (robList->robots[i].xm1==HUGE_VAL){
	PRINTW("ROBOT %d misura assente\n",robList->robots[i].id);
}
else if (robList->robots[i].xm1==-HUGE_VAL){
	PRINTW("ROBOT %d misura non valida\n",robList->robots[i].id);
}
	else PRINTW("ROBOT %d misura valida\n",robList->robots[i].id);
}*/


//==============================================================================
//Calcola le coordinate x,y e th, associate ad una coppia di marker
/*
 * NUOVO
 * 									 _______
 *			 90|					|		|
 *			   |					|	R	|
 *	180________|________0			|	*	|	* ---> coordinate robot (punto medio)
 *			   |					|	G	|			R = m1
 *			   |					|_______|			G = m2
 *			270|-90 
 */ 		


//int GetColorBlob(colorBlobList **colorBList, colorRobotList **colorRList, colorMarkerList **colorMList, ComponentsList *compList, PixelMap *pixelMap, FILE *log_file) {
int GetColorBlob(colorBlobList **colorBList,ComponentsList *compList, PixelMap *pixelMap, FILE *log_file) {
	
struct timeval time;
long timestamp;
int i,centroidIndex;
int imageRows,imageCols;

//int j=0, k=0, m=0, numBlob=0;//GIO
int j=0, numBlob=0;//GIO
//int ri, ii, jj;
//int ci=0; 
//float d1,d2,d3,d4;		
	
imageRows = pixelMap->rows;
imageCols = pixelMap->cols;
	
//printf("Col: %d\tRow: %d\n",imageCols,imageRows);
ComponentsListElement*  compListElement;
Component* comp;
	
compListElement = compList->head;
	
//Per distinguere i robot individuati: HUGE_VAL ----> non individuato 
/*for(i=0;i<numRobots;i++){
	robList->robots[i].xm1=HUGE_VAL;
	robList->robots[i].ym1=HUGE_VAL;
	robList->robots[i].xm2=HUGE_VAL;
	robList->robots[i].ym2=HUGE_VAL;		
	robList->robots[i].x=-HUGE_VAL;
	robList->robots[i].y=-HUGE_VAL;
	robList->robots[i].distance=-HUGE_VAL;
	robList->robots[i].orientation=-HUGE_VAL;
}*/

//Dimensiono la struttura a seconda dei blob individuati//GIO
for(i=0;i<compList->count;i++)	{
	comp = &compListElement->component;
		
	// Get centroid distance only if the blob is big enough
	if ( (comp->pixelList->count>BLOB_MIN_SIZE) && (comp->pixelList->count<BLOB_MAX_SIZE) ) {
		numBlob++;		
	}
	compListElement = compListElement->next;
}

//Creo la lista dei blob e cancello quella precedentemente creata
if(*colorBList != NULL) {
	if((*colorBList)->blobs != NULL ) {
		free((*colorBList)->blobs);			
	}
	free(*colorBList);
}
	
*colorBList = (colorBlobList *)malloc( sizeof( colorBlobList ) );
(*colorBList)->blobs = (colorBlob *)malloc(sizeof(colorBlob)*numBlob);
/*	
if(*commonMList != NULL) {
	if((*commonMList)->markers != NULL ) {
		free((*commonMList)->markers);			
	}
	free(*commonMList);
}
	
*commonMList = (commonMarkerList *)malloc( sizeof( commonMarkerList ) );
(*commonMList)->markers = (commonMarker *)malloc(sizeof(commonMarker)*numBlob);

(*commonMList)->numMarker=numBlob;
int ind;
for(ind=0;ind<numBlob;ind++){
	(*commonMList)->markers[ind].markerX=HUGE_VAL;
}


if(*robList != NULL) {
	if((*robList)->robots != NULL) {
		free((*robList)->robots);
	}
	free(*robList);
}

*robList = (robotList *)malloc( sizeof( robotList ) );
(*robList)->numRob=numRobots;
(*robList)->robots = (robot *)malloc(sizeof(robot)*numRobots);


for(ind=0;ind<numRobots;ind++){
	(*robList)->robots[ind].id=ind+1;
(*robList)->robots[ind].xm1=HUGE_VAL;
(*robList)->robots[ind].ym1=HUGE_VAL;
(*robList)->robots[ind].xm2=-HUGE_VAL;
(*robList)->robots[ind].ym2=-HUGE_VAL;		
(*robList)->robots[ind].x=-HUGE_VAL;
(*robList)->robots[ind].y=-HUGE_VAL;
(*robList)->robots[ind].distance=-HUGE_VAL;
(*robList)->robots[ind].orientation=-HUGE_VAL;
}*/


//debug
//PrintPixelMap(pixelMap);
		
gettimeofday( &time, NULL );
timestamp =  ((time.tv_sec) * 1000000 + time.tv_usec);

compListElement = compList->head;	
//for each component
	

// printf("component count: %d\n",compList->count); // Andrea
for(i=0;i<compList->count;i++)	{

	comp = &compListElement->component;
		
	// Get centroid distance only if the blob is big enough
		
	if ( (comp->pixelList->count>BLOB_MIN_SIZE) && (comp->pixelList->count<BLOB_MAX_SIZE) ) {
		
		centroidIndex = comp->centroidRow*imageCols + comp->centroidCol;
		#ifdef PRINT_VISION
		printf("ROW:%d COL:%d\n",comp->centroidRow,comp->centroidCol);
		#endif
		//QUI//printf("Index: %d\n",centroidIndex);
		//coordinate cartesiane
		
		
		//ATTILIO: THIS HACK IS REQUIRED TO DEAL WITH 720p CAMERA
		//BY CREATIVE
		comp->centroidX = pixelMap->x_coord[centroidIndex]/10;
		comp->centroidY = pixelMap->y_coord[centroidIndex]/10;
		//coordinate polari [da migliorare->atan2] //GIO		
		comp->centroid_distance=GetDistCentroidBlob(comp->centroidX,comp->centroidY);
		comp->centroid_angle=GetAngleCentroidBlob(comp->centroidX,comp->centroidY);
		//printf("Cartesian [coordinates]/pixelIndex of component %d centroid :  [%f , %f]/%d  \n",i,comp->centroidX,comp->centroidY,centroidIndex);
		//printf("component %d centroid with color %d [X,Y] = [%f ,%f] \n",i,comp->color,comp->centroidX,comp->centroidY);
		//STAMPA QUESTO
		#ifdef PRINT_VISION
		printf("Component %d, color %d [X,%f ,Y,%f , Distance,%f , Angle,%f] \n",i,comp->color,comp->centroidX,comp->centroidY,comp->centroid_distance,comp->centroid_angle);
		#endif
		//printf("\n\nCOMMON COLOR: %d\n\n", commonColorId);		
		//Memorizzo nella struttura colorBlob le informazioni ricavate//GIO
		(*colorBList)->blobs[j].color=comp->color;
		(*colorBList)->blobs[j].centroidX=comp->centroidX;
		(*colorBList)->blobs[j].centroidY=comp->centroidY;
		(*colorBList)->blobs[j].centroid_distance=comp->centroid_distance;
		(*colorBList)->blobs[j].centroid_angle=comp->centroid_angle;			
		/*//DA QUI vecchio
		//HUGE_VAL ---->robot non individuato
		//-HUGE_VAL ---->misura non attendibile
		int ri=(*colorBList)->blobs[j].color-2;	//indice robot= colore blob -2
		
		if((ri+2)!=commonColorId){	//se non è il colore comune
			//PRINTE("%d\n",ri);
			//printRobot(ri,*robList);
			if((*robList)->robots[ri].xm1==HUGE_VAL){	//se non ho ancora trovato un marker di quel colore aggiorno la misura
								
				
				(*robList)->robots[ri].xm1=(*colorBList)->blobs[j].centroidX;
				(*robList)->robots[ri].ym1=(*colorBList)->blobs[j].centroidY;
				//printRobot(ri,*robList);
				
			}
			else{	//altrimenti annullo la misura
				if((*robList)->robots[ri].xm1!=-HUGE_VAL){ //se la misura non è stata scartata
					if(fabs((*robList)->robots[ri].xm1-(*colorBList)->blobs[j].centroidX)<20){	//se due blob dello stesso colore sono vicini considero il loro baricentro
						(*robList)->robots[ri].xm1=((*robList)->robots[ri].xm1+(*colorBList)->blobs[j].centroidX)/2;
						(*robList)->robots[ri].ym1=((*robList)->robots[ri].ym1+(*colorBList)->blobs[j].centroidY)/2;	
					}
					else{	//altrimenti considero non attendibile la misura
							(*robList)->robots[ri].xm1=-HUGE_VAL;
							(*robList)->robots[ri].ym1=-HUGE_VAL;
						}				
					}
				
				}	//printRobot(ri,*robList); CONSIDERARE CHE IL ROSSO PUÒ ESSERE IL PRIMO AD ESSERE VISTO E QUINDI IL MARKER 1 È ANCORA HUGE_VAL 	
		}

			else{	//se è il colore comune
				//lo aggiungo alla lista 
				(*commonMList)->markers[ci].markerX=(*colorBList)->blobs[j].centroidX;
				(*commonMList)->markers[ci].markerY=(*colorBList)->blobs[j].centroidY;	
				(*commonMList)->markers[ci].marker_distance=(*colorBList)->blobs[j].centroid_distance;
				ci++;
				for(ii=0;ii<numRobots;ii++){
					
					//d4 = distanza marker1/origine
					d4=fabs(sqrt(((*robList)->robots[ii].xm1*(*robList)->robots[ii].xm1)+((*robList)->robots[ii].ym1*(*robList)->robots[ii].ym1)));
					//d3 = distanza vecchio marker2/origine
					d3=fabs(sqrt(((*robList)->robots[ii].xm2*(*robList)->robots[ii].xm2)+((*robList)->robots[ii].ym2*(*robList)->robots[ii].ym2)));	
					//d1 = distanza marker1/nuovo marker2
					d1=fabs(d4-(*colorBList)->blobs[j].centroid_distance);	
					//d2 = distanza marker1/vecchio marker2
					d2=fabs(d4-d3);		
					if(d1>d2){	//se il nuovo marker è più vicino aggiorno
						(*robList)->robots[ii].xm2=(*colorBList)->blobs[j].centroidX;
						(*robList)->robots[ii].ym2=(*colorBList)->blobs[j].centroidY;					
					}
					
				}
			} 	*/			
	j++;
			// If required, save coordinates in text file
			if (LOGGING){
			// Andrea 			
			 LogCoordDataEvo(comp->centroidX,comp->centroidY,timestamp,compList->count,comp->color);
			 //fprintf(cam_log_file,"\t %f \t %f \t %ld \t %d\n",comp->centroidX,comp->centroidY,timestamp,compList->count );
			}	
		}	
		compListElement = compListElement->next;
		
		if(numBlob) //GIO
		(*colorBList)->numBlob = numBlob;
}

	
	// Andrea 
	
	if (compList->count>0)
		ClearComponentsList(compList);
	
	return 0;
}

//==============================================================================

int PrintPixelMap(PixelMap *pixelMap){
	
	int i;
	#ifdef PRINT_VISION
	printf("PixelMap rows : %d\n",pixelMap->rows);
	printf("PixelMap cols : %d\n",pixelMap->cols);
	printf("PixelMap coordinates : \n");
	
	for(i=0;i<pixelMap->rows*pixelMap->cols;i++){
		printf("Pixel [index]/(coordinates) = [%d]/(%f,%f)\n",i,pixelMap->x_coord[i],pixelMap->y_coord[i]);
	}
	#endif
	return 0;
}


//==============================================================================

float GetDistCentroidBlob(float x_cent_blob, float y_cent_blob)
{
	if (y_cent_blob==0.0)
	{
		return x_cent_blob;
	} else 
	{
		return (sqrt(pow(x_cent_blob,2.0)+pow(y_cent_blob,2.0)));
	}
}

//==============================================================================

float GetAngleCentroidBlob(float x_cent_blob, float y_cent_blob) {

//return atan2(-1*y_cent_blob,x_cent_blob);
float result= atan2(y_cent_blob,x_cent_blob);
if(isnan(result))
  printf("Error on centroid's angle. It is nan\n");
return result;
//void prova(colorBlobList *colorBList){
//printf("Prova: %f\n", colorBList->blobs[0].centroidX);
//printf("Prova: %f\n", colorBList->blobs[0].centroidY);
//}
}

//==============================================================================




