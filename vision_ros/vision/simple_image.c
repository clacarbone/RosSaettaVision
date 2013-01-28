//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Format conversion and simple image handling functions
//					implementations
//
//  Description:    Provides a high-level C interface for controlling frame
//                  grabber. Based on uvc streamer, supports v4l2 and UVC driver
//
//  Author:         Enrico Di Lello
//  Author:  	    and many small changes by Giovanni Micheli
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
//==========================================================================

//#include "simple_image.h" 
#include "globals.h" 


//=============================================================================

int CreateColorStruct(ColorStruct **colorStruct) {
	// alloco la struttura ColorStruct
	*colorStruct = (ColorStruct *)malloc( sizeof( ColorStruct ) );
	//int commonColorId;
	//commonColor=(CommonColor*)malloc(sizeof(CommonColor));	
	
	if ( *colorStruct == NULL)
	{
		printf("CreateColorStruct: Non è possibile creare colorStruct (out of memory?)...\n");
		return -1;
	}
	FILE *ff;
    	ff=fopen(COLOR_FILE,"r");
    	if(ff==NULL) 
		printf("Error on file opening\n");
	//numero massimo di colori disponibili e numero di colori da utilizzare
	fscanf(ff,"%d\n%d\n",&((*colorStruct)->maxColor),&((*colorStruct)->numColor));
	int i,j;
	i=0;
	(*colorStruct)->useColor = (int*)malloc(sizeof(int)*((*colorStruct)->numColor));
	(*colorStruct)->dtColorYCbCr = (int*)malloc(sizeof(int)* 6 * (*colorStruct)->maxColor );
	
	for(j=0;j<(*colorStruct)->numColor;j++){
	//identificativo colore
	(*colorStruct)->useColor[j]=j+1;
	//codifica colri YCbCr
	fscanf(ff,"%d\t%d\t%d\t%d\t%d\t%d\n",&((*colorStruct)->dtColorYCbCr[i]),&((*colorStruct)->dtColorYCbCr[i+1]),&((*colorStruct)->dtColorYCbCr[i+2]),&((*colorStruct)->dtColorYCbCr[i+3]),&((*colorStruct)->dtColorYCbCr[i+4]),&((*colorStruct)->dtColorYCbCr[i+5]));
	i=i+6;
	}
	i=0;
	(*colorStruct)->dtColorRGB = (int*)malloc(sizeof(int)* 3 * (*colorStruct)->maxColor);
	//codifica colori RGB
	for(j=0;j<(*colorStruct)->numColor;j++){
		fscanf(ff,"%d\t%d\t%d\t",&((*colorStruct)->dtColorRGB[i]),&((*colorStruct)->dtColorRGB[i+1]),&((*colorStruct)->dtColorRGB[i+2]));
		i=i+3;
	}
	fclose(ff);
	//i=0;
/*
	commonColor->colorId=commonColorId;
	for(i=0;i<6;i++)	
		commonColor->colorYCbCr[i]=(*colorStruct)->dtColorYCbCr[6*(commonColorId-1)+i];
	for(i=0;i<3;i++)
		commonColor->colorRGB[i]=(*colorStruct)->dtColorRGB[3*(commonColorId-1)+i];
	i=0;
*/
	#ifdef PRINT_VISION
	//Stampa codifica colori
	printf("COLOR PARAM:\n");
	printf("max_color\t%d\nnum_color\t%d\n",(*colorStruct)->maxColor,(*colorStruct)->numColor);
	printf("color id:\n\t%d\n\t%d\n",(*colorStruct)->useColor[0],(*colorStruct)->useColor[1]);
	printf("RED\n\tYCbCr\n");
	printf("\t\t%d\t%d\t%d\t%d\t%d\t%d\n",(*colorStruct)->dtColorYCbCr[0],(*colorStruct)->dtColorYCbCr[1],(*colorStruct)->dtColorYCbCr[2],(*colorStruct)->dtColorYCbCr[3],	(*colorStruct)->dtColorYCbCr[4],(*colorStruct)->dtColorYCbCr[5]);
	printf("GREEN\n\tYCbCr\n");
	printf("\t\t%d\t%d\t%d\t%d\t%d\t%d\n",(*colorStruct)->dtColorYCbCr[6],(*colorStruct)->dtColorYCbCr[7],(*colorStruct)->dtColorYCbCr[8],(*colorStruct)->dtColorYCbCr[9],	(*colorStruct)->dtColorYCbCr[10],(*colorStruct)->dtColorYCbCr[11]);
	
	printf("RED\n\tRGB\n");
	printf("\t\t%d\t%d\t%d\n",(*colorStruct)->dtColorRGB[0],(*colorStruct)->dtColorRGB[1],	(*colorStruct)->dtColorRGB[2]);
	printf("GREEN\n\tRGB\n");
	printf("\t\t%d\t%d\t%d\n",(*colorStruct)->dtColorRGB[3],(*colorStruct)->dtColorRGB[4],	(*colorStruct)->dtColorRGB[5]);
	#endif
	return 0;
	}
//=============================================================================

void DestroyColorStruct(ColorStruct *colorStruct) {
	free( colorStruct->dtColorYCbCr );
	free( colorStruct->dtColorRGB );
	free( colorStruct );
}
//=============================================================================

int ProcessFrameEvol(){
		
	// Apply a color threshold to the frame and put the result in a 
	// matrix which represent a colors codified image
	#ifdef PRINT_VISION	
	#ifdef VISION_PRINT	
		printf("Thresholding frame number %d\n",frame_counter);
	#endif
	#endif
	gettimeofday(&step_start_time,NULL);
		
	if ( ThresholdFrameMultiColor(fr, bwIm) != 0 )  {
		return -1;
	}
		
	// If required, save one out of n_save_frame colors codified images in a .ppm file
	if ( SAVE_COLOR_IMG ) {	
		if ( (frame_counter%n_save_frame) == 0){
			snprintf( binary_fname, sizeof(binary_fname), "color_%d.ppm",frame_counter );
			SaveColorImage(bwIm, binary_fname);
			#ifdef PRINT_VISION
			printf("Color image %d saved to file \n",frame_counter);  
			#endif 
		}
	}	
		
	// Label the connected components of the image
	#ifdef PRINT_VISION
	#ifdef VISION_PRINT
		printf("Labelling image number %d\n",frame_counter);	
	#endif	
	#endif
		
	if ( CreateComponentsList(&compList,2) != 0 ) {
		return -1;
	}
		
	if ( PerformLabellingColor(bwIm,compList, colorStruct->maxColor) != 0 )	{
		return -1;
	}
		
	// Compute blob centroids
	if ( ComputeCentroidsColor(compList,bwIm) != 0 ) {
		return -1;
	}		
		
	gettimeofday(&step_end_time, NULL);
	GetElapsedTime("Frame Processing",&step_start_time, &step_end_time, &processing_time);
		
	if ( SAVE_CENTROIDS ) {	
		if ( (frame_counter%n_save_frame) == 0){
			snprintf( binary_fname, sizeof(binary_fname), "color_%d.ppm",frame_counter );
			SaveColorImage(bwIm, binary_fname);
			#ifdef PRINT_VISION
			printf("Color image %d saved to file \n",frame_counter);   
			#endif
		}
	}	
		
	// Clear dynamic data structures
		
		//QUI
		//DestroyColorImage (bwIm);
		//ReleaseFrame(&fr);
	return 0;
}
//=============================================================================

int ThresholdFrameMultiColor(FRAME *fr, BWImage *bwIm){
		
	int i,imageLength; 
	int indexUseColor, codColor;
	int PixelI,PixelJ;
	unsigned char Y=0;
	unsigned char Cr=0,Cb=0;


	ymin=crmin=cbmin=255;
	ymax=crmax=cbmax=0;

	soglie = fopen(FILE_SOGLIE,"a");
		
	//Add one line of black pixels at the beginning for centroid extraction step
	for (i=0; i<bwIm->cols;i++){
		bwIm->data[i] = 0;
	}
		
	// Scan the original frame 
	for ( i=0; i < (fr->width * fr->height); i++) {
		//printf("\n\n\n%d\n\n\n",i);
		//=================================================
		//		Color segmentation step					  
		//=================================================
		if (fr->format == V4L2_PIX_FMT_YUYV)	{
			
			// Get the Y component of the required pixel
			Y = ((unsigned char*)(fr->data))[i*2];
			//Get the Cr component of the required pixel
			//aggiungo la componente Cb
			if (i%2 == 0) {
				Cr = ((unsigned char*)(fr->data))[(i*2)+3];
				Cb = ((unsigned char*)(fr->data))[(i*2)+1];				
			}
			else {
				Cr = ((unsigned char*)(fr->data))[(i*2)+1];
				Cb = ((unsigned char*)(fr->data))[(i*2)-1];
			}
			
			
		
		}
			
		if (fr->format == VIDEO_PALETTE_YUV420P)	{
			imageLength = fr->width*fr->height;
			//Get 2 dimensional pixel indexes
			PixelI = i/fr->width;
			PixelJ = i%fr->width;
				
			//Get the Y component of the required pixel
			Y =  ((unsigned char*)(fr->data))[i];
			
			//Get the Cr component of the required pixel
			Cr = ((unsigned char*)(fr->data))[(PixelI/2*(fr->width/2)) + PixelJ/2 + imageLength + imageLength/4]; 
		}						
		bwIm->data[(i)+bwIm->cols] = 0;
		bwIm->data[(10)+bwIm->cols]=1;
		bwIm->data[(11)+bwIm->cols]=2;
		bwIm->data[(12)+bwIm->cols]=3;
		bwIm->data[(13)+bwIm->cols]=4;
		// Per ogni pixel controllo i valori di soglia specifici di ogni colore di interesse
		


for (indexUseColor=0; indexUseColor < colorStruct->numColor; indexUseColor++) {
			//individuo il codice colore nel vettore dei colori da utilizzare
			codColor = colorStruct->useColor[indexUseColor] ;
				
			//verifico le soglie per ogni codice colore selezionato
//*



			if ((Y>dtYmin(codColor))&&(Y<dtYmax(codColor))&&(Cb>dtCbmin(codColor))&&(Cb<dtCbmax(codColor))&&(Cr>dtCrmin(codColor))&&(Cr<dtCrmax(codColor))){
				bwIm->data[(i)+bwIm->cols] = codColor;

			}


			
//*/			
//verifico le soglie per ogni codice colore selezionato
//9	134	144	185	106	129

		/*	if ((Y>9) && (Y<134) && (Cr>144) && (Cr<185) && (Cb>109) && (Cb<129)){

				bwIm->data[(i)+bwIm->cols] = 1;//codColor;
					
			}

			if ((Y>60) && (Y<105) && (Cr>100) && (Cr<113) && (Cb>130) && (Cb<154)){

				bwIm->data[(i)+bwIm->cols] = 2;//codColor;
				
			}
			
			if ((Y>92) && (Y<113) && (Cr>113) && (Cr<123) && (Cb>111) && (Cb<124)){

			bwIm->data[(i)+bwIm->cols] = 3;//codColor;
				
			}

			if ((Y>108) && (Y<116) && (Cr>127) && (Cr<131) && (Cb>96) && (Cb<113)){

			bwIm->data[(i)+bwIm->cols] = 4;//codColor;
				
			}*/
		}

 
/*	
int kk=0;
int a1,a2,a3,a4;
int b1,b2,b3,b4;
int c1,c2,c3,c4;
int d1,d2,d3,d4;
int e1,e2,e3,e4;
int f1,f2,f3,f4;
//*/
/*RED
a1=294;
a2=199;
a3=306;
a4=203;
//*/
/*
b1=231;
b2=124;
b3=239;
b4=126;

c1=231;
c2=124;
c3=239;
c4=126;

d1=231;
d2=124;
d3=239;
d4=126;

e1=231;
e2=124;
e3=239;
e4=126;

f1=231;
f2=124;
f3=239;
f4=126;


//*/

/*CYAN
a1=208;
a2=77;
a3=215;
a4=80;

b1=376;
b2=84;
b3=384;
b4=87;

c1=276;
c2=120;
c3=285;
c4=124;

d1=199;
d2=165;
d3=208;
d4=170;

e1=339;
e2=195;
e3=350;
e4=201;

f1=339;
f2=195;
f3=350;
f4=201;
//*/

/*GREEN
a1=401;
a2=104;
a3=411;
a4=109;

b1=208;
b2=124;
b3=216;
b4=129;

c1=300;
c2=127;
c3=307;
c4=131;

d1=304;
d2=226;
d3=315;
d4=232;

e1=215;
e2=185;
e3=225;
e4=190;

f1=288;
f2=337;
f3=301;
f4=345;

//*/

/*YELLOW
a1=299;
a2=83;
a3=308;
a4=87;

b1=299;
b2=83;
b3=308;
b4=87;

//b1=211;
//b2=174;
//b3=219;
//b4=180;

c1=376;
c2=179;
c3=385;
c4=184;

d1=261;
d2=272;
d3=275;
d4=282;

e1=187;
e2=189;
e3=195;
e4=193;

f1=187;
f2=189;
f3=195;
f4=193;
//*/

/*
int a5=a4-a2+1;
int b5=b4-b2+1;
int c5=c4-c2+1;
int d5=d4-d2+1;
int e5=e4-e2+1;
int f5=f4-f2+1;

for(kk=0;kk<a5;kk++)
	if((i>a1+(a2+kk)*640) && (i<a3+(a2+kk)*640)){
		//bwIm->data[(i)+bwIm->cols] = 1;
		printf("GREEN: Y: %d, Cb: %d, Cr: %d\n",Y,Cb,Cr);
		
		if(Y<ymin) ymin=Y;
		if(Y>ymax) ymax=Y;
		if(Cr<crmin) crmin=Cr;
		if(Cr>crmax) crmax=Cr;
		if(Cb<cbmin) cbmin=Cb;
		if(Cb>cbmax) cbmax=Cb;
		
	}
if(i==a3+a4*640){
printf("\nSOGLIE:\t%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
fprintf(soglie,"%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
//fclose(soglie);

}
//*/
/*
for(kk=0;kk<b5;kk++)
	if((i>b1+(b2+kk)*640) && (i<b3+(b2+kk)*640)){
		//bwIm->data[(i)+bwIm->cols] = 1;
		printf("GREEN: Y: %d, Cb: %d, Cr: %d\n",Y,Cb,Cr);
		
		if(Y<ymin) ymin=Y;
		if(Y>ymax) ymax=Y;
		if(Cr<crmin) crmin=Cr;
		if(Cr>crmax) crmax=Cr;
		if(Cb<cbmin) cbmin=Cb;
		if(Cb>cbmax) cbmax=Cb;
		
	}
if(i==b3+b4*640){
printf("\nSOGLIE:\t%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
fprintf(soglie,"%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
//fclose(soglie);

}

for(kk=0;kk<c5;kk++)
	if((i>c1+(c2+kk)*640) && (i<c3+(c2+kk)*640)){
		//bwIm->data[(i)+bwIm->cols] = 1;
		printf("GREEN: Y: %d, Cb: %d, Cr: %d\n",Y,Cb,Cr);
		
		if(Y<ymin) ymin=Y;
		if(Y>ymax) ymax=Y;
		if(Cr<crmin) crmin=Cr;
		if(Cr>crmax) crmax=Cr;
		if(Cb<cbmin) cbmin=Cb;
		if(Cb>cbmax) cbmax=Cb;
		
	}
if(i==c3+c4*640){
printf("\nSOGLIE:\t%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
fprintf(soglie,"%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
//fclose(soglie);

}

for(kk=0;kk<d5;kk++)
	if((i>d1+(d2+kk)*640) && (i<d3+(d2+kk)*640)){
		//bwIm->data[(i)+bwIm->cols] = 1;
		printf("GREEN: Y: %d, Cb: %d, Cr: %d\n",Y,Cb,Cr);
		
		if(Y<ymin) ymin=Y;
		if(Y>ymax) ymax=Y;
		if(Cr<crmin) crmin=Cr;
		if(Cr>crmax) crmax=Cr;
		if(Cb<cbmin) cbmin=Cb;
		if(Cb>cbmax) cbmax=Cb;
		
	}
if(i==d3+d4*640){
printf("\nSOGLIE:\t%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
fprintf(soglie,"%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
//fclose(soglie);

}

for(kk=0;kk<e5;kk++)
	if((i>e1+(e2+kk)*640) && (i<e3+(e2+kk)*640)){
		//bwIm->data[(i)+bwIm->cols] = 1;
		printf("GREEN: Y: %d, Cb: %d, Cr: %d\n",Y,Cb,Cr);
		
		if(Y<ymin) ymin=Y;
		if(Y>ymax) ymax=Y;
		if(Cr<crmin) crmin=Cr;
		if(Cr>crmax) crmax=Cr;
		if(Cb<cbmin) cbmin=Cb;
		if(Cb>cbmax) cbmax=Cb;
		
	}
if(i==e3+e4*640){
printf("\nSOGLIE:\t%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
fprintf(soglie,"%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
//fclose(soglie);

}

for(kk=0;kk<f5;kk++)
	if((i>f1+(f2+kk)*640) && (i<f3+(f2+kk)*640)){
		//bwIm->data[(i)+bwIm->cols] = 1;
		printf("GREEN: Y: %d, Cb: %d, Cr: %d\n",Y,Cb,Cr);
		
		if(Y<ymin) ymin=Y;
		if(Y>ymax) ymax=Y;
		if(Cr<crmin) crmin=Cr;
		if(Cr>crmax) crmax=Cr;
		if(Cb<cbmin) cbmin=Cb;
		if(Cb>cbmax) cbmax=Cb;
		
	}
if(i==f3+f4*640){
printf("\nSOGLIE:\t%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
fprintf(soglie,"%d\t%d\t%d\t%d\t%d\t%d\n",ymin,ymax,cbmin,cbmax,crmin,crmax);
//fclose(soglie);

}

//*/
/*	if((i>213+151*640) && (i<213+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>214+151*640) && (i<214+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>215+151*640) && (i<215+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>216+151*640) && (i<216+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>217+151*640) && (i<217+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>218+151*640) && (i<218+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>219+151*640) && (i<219+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);
	if((i>220+151*640) && (i<220+155*640))
		printf("GREEN: Y: %d, Cr: %d, Cb: %d\n",Y,Cr,Cb);*/
}
fclose(soglie);
	return 0;
}
//=============================================================================

int CreateBWImage( BWImage **im, int rows, int cols )
	{
		*im = (BWImage *)malloc( sizeof( BWImage ) );
		
		if ( *im == NULL)
		{
			printf("CreateBWImage: Cannot create bwIm (out of memory?)...\n");
			return -1;
		}
	    
		(*im)->rows = rows;
		(*im)->cols = cols;
		(*im)->data = (unsigned char*)malloc((rows*cols)*sizeof(unsigned char));
		
		if ( (*im)->data == NULL)
		{
			printf("CreateBWImage: Cannot create data field (out of memory?)...\n");
			return -1;
		}
		
		return 0;
	}
//==============================================================================

void DestroyBWImage( BWImage* im ){
	free( im->data );
	free( im );
}
//=============================================================================

int SaveBWImage ( BWImage *bwIm, const char* filename ){
	int i = 0;
	FILE* fp = fopen( filename, "w" );
		
	if ( fp == NULL ){
		perror( "SaveBWImage: opening file for writing" );
		return -1;
	}
		
	// Write PNM header
	fprintf( fp, "P6\n" );
	fprintf( fp, "# Generated by a herd of rabid fleas\n" );
	fprintf( fp, "%d %d\n", bwIm->cols, bwIm->rows );
		
	// Max val
	fprintf( fp, "255\n" );
		
	for ( i=0; i < (bwIm->rows * bwIm->cols); i++) {
		if(bwIm->data[i] == 1) {
			fprintf(fp,"%c%c%c", 255,255,255);
		}
		else if (bwIm->data[i] == RED){
			//printf("SaveBWImage : Centroid pixel marked \n");
			//getchar();
			fprintf( fp, "%c%c%c",255,0,0 ); 
			}
		     else {
			fprintf(fp,"%c%c%c", 0,0,0);
		     }
	}
	fclose( fp );
	return 0;
}
//=============================================================================

void DestroyColorImage( BWImage* im )	{
	free( im->data );
	free( im );
}
//=============================================================================

int SaveColorImage ( BWImage *bwIm, const char* filename ){
	int i = 0;
	int codColor = 0;
	FILE* fp = fopen( filename, "w" );
		
	if ( fp == NULL ){
		perror( "SaveColorImage: opening file for writing" );
		return -1;
	}
		
	// Write PNM header
	fprintf( fp, "P6\n" );
	fprintf( fp, "# Generated by a herd of rabid fleas\n" );
	fprintf( fp, "%d %d\n", bwIm->cols, bwIm->rows );
		
	// Max val
	fprintf( fp, "255\n" );
		
	for ( i=0; i < (bwIm->rows * bwIm->cols); i++) {
		// memorizzo il codice colore relativo al pixel
		codColor = bwIm->data[i];
		// se il codice corrisponde ad uno dei colori 
		if (codColor < colorStruct->maxColor+1 && codColor>0 ) {
			// salvo nell'immagine l'rgb corrispondente al codice colore
			fprintf(fp,"%c%c%c", dtR(codColor),dtG(codColor),dtB(codColor));
		} 
		// se invece corrisponde al codice della croce del centroide
		else if (codColor == CROSS){
			fprintf(fp, "%c%c%c",255,255,255); //coloro la croce con il bianco
		} 
		// altrimenti il pixel corrispondente verra' rappresentato in nero		
	             else 
		     	fprintf(fp,"%c%c%c", 0,0,0);
		
		}
		fclose( fp );
	return 0;
}
//=============================================================================

inline int YUV422toRGB(FRAME_RGB* pixelRGB, FRAME* frame, int pixelIndex)
{	
	unsigned char Y;
	unsigned char Cb;
	unsigned char Cr;
	int R,G,B;
	char U;
	char V;

	
	//Get the Y component of the required pixel
	Y =  ((unsigned char*)(frame->data))[(pixelIndex*2)];
	
	//Get the Cb and Cr components of the required pixel
	
	if (pixelIndex%2 == 0) {
		Cb = ((unsigned char*)(frame->data))[(pixelIndex*2)+1];
		Cr = ((unsigned char*)(frame->data))[(pixelIndex*2)+3];
	}
	else {
		Cb = ((unsigned char*)(frame->data))[(pixelIndex*2)-1];
		Cr = ((unsigned char*)(frame->data))[(pixelIndex*2)+1]; 
	}
	
	//Convert YCbCr to YUV
	
	CLAMP_YCbCr(&Y,&Cb,&Cr);        // Clamp the values to their valid range  
	
	Y = YtoY1(Y - 16);
	U = CbtoU(Cb - 128);
	V = CrtoV(Cr - 128);
	
	//CLAMP_YUV(&Y,&U,&V);
	
	//Convert YUV to RGB values     (http://www.thedirks.org/v4l2/v4l2fmt.htm)
	
	R = YUVtoR(Y,V);
	G = YUVtoG(Y,U,V);
	B = YUVtoB(Y,U);
	
	CLAMP_RGB(&R,&G,&B);			// Clamp the values to their valid range 
	
	pixelRGB->red = (unsigned char)(R);
	pixelRGB->green = (unsigned char)(G); 
	pixelRGB->blue = (unsigned char)(B);
	
	return 0;
}

//=============================================================================

inline int YUV420PtoRGB(FRAME_RGB* pixelRGB,FRAME* frame, int pixelIndex) {
	
	unsigned char Y;
	unsigned char Cb;
	unsigned char Cr;
	
	int R,G,B;
	char U;
	char V;
	
	int imageLength;
	imageLength = frame->width*frame->height;
	int PixelI;
	int PixelJ;
	
	//Get 2 dimensional pixel indexes
	PixelI = pixelIndex/frame->width;
	PixelJ = pixelIndex%frame->width;
	
	//Get the Y component of the required pixel
	Y =  ((unsigned char*)(frame->data))[pixelIndex];
	
	//Get the Cb component of the required pixel
	Cb = ((unsigned char*)(frame->data))[(PixelI/2*(frame->width/2)) + PixelJ/2 + imageLength];
	
	//Get the Cr component of the required pixel
	Cr = ((unsigned char*)(frame->data))[(PixelI/2*(frame->width/2)) + PixelJ/2 + imageLength + imageLength/4]; 
	
	//Convert YCbCr to YUV
	
	CLAMP_YCbCr(&Y,&Cb,&Cr);
	
	Y = YtoY1(Y - 16);
	U = CbtoU(Cb - 128);
	V = CrtoV(Cr - 128);
	
	//Convert YUV to RGB values (http://www.thedirks.org/v4l2/v4l2fmt.htm)
	
	R = YUVtoR(Y,V);
	G = YUVtoG(Y,U,V);
	B = YUVtoB(Y,U);
	
	CLAMP_RGB(&R,&G,&B);
	
	pixelRGB->red = (unsigned char)(R);
	pixelRGB->green = (unsigned char)(G); 
	pixelRGB->blue = (unsigned char)(B);
	
	
	
	return 0;
}

/*	
//=================================================================================

inline int RGBtoYUV(FRAME_RGB* pixelRGB) 
{
	
	unsigned char Y;
	unsigned char U;
	unsigned char V;
	
	//Convert RGB to YUV values (http://www.thedirks.org/v4l2/v4l2fmt.htm)
	
	Y = RGBtoY(pixelRGB->red,pixelRGB->green,pixelRGB->blue);		
	U = RGBtoU(pixelRGB->red,pixelRGB->green,pixelRGB->blue);
	V = RGBtoV(pixelRGB->red,pixelRGB->green,pixelRGB->blue);
	
	pixelRGB->red = Y;
	pixelRGB->green = U;
	pixelRGB->blue = V;
	
	return 0;
}*/
//=============================================================================

inline int CLAMP_YCbCr(unsigned char* Y_p, unsigned char* Cb_p, unsigned char* Cr_p){

	if (*Y_p<=16)   *Y_p = 16;
	if (*Y_p>=235)  *Y_p = 235;
	if (*Cb_p<=16)  *Cb_p = 16;
	if (*Cb_p>=240) *Cb_p = 240; 
	if (*Cr_p<=16)  *Cr_p = 16;
	if (*Cr_p>=240) *Cr_p = 240;
	return 0;
} 

//=============================================================================

inline int CLAMP_YUV(unsigned char* Y_p, unsigned char* U_p, unsigned char* V_p){

	if (*Y_p<=0)   *Y_p = 0;
	if (*Y_p>=255) *Y_p = 255;
	if (*U_p<=0)   *U_p = 0;
	if (*U_p>=255) *U_p = 255; 
	if (*V_p<=0)  *V_p = 0;
	if (*V_p>=255) *V_p = 255;
	return 0;
} 
//=============================================================================

inline int CLAMP_RGB(int* R_p, int* G_p, int* B_p){
	
	if (*R_p<=0)   *R_p = 0;
	if (*R_p>=255) *R_p = 255;
	if (*G_p<=0)  *G_p = 0;
	if (*G_p>=255) *G_p = 255; 
	if (*B_p<=0)  *B_p = 0;
	if (*B_p>=255) *B_p = 255;
	return 0;
}
//=============================================================================

int ResolveFrameDimensions(char* size, int* frame_width_pt, int* frame_height_pt){
	
	if ( strcmp(size,"sqcif") == 0 )
	{
		*frame_width_pt = 128;
		*frame_height_pt = 96;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}
	
	else if ( strcmp(size,"qsif") == 0 )
	{
		*frame_width_pt = 160;
		*frame_height_pt = 120;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}
	
	else if ( strcmp(size,"qcif") == 0 )
	{
		*frame_width_pt = 176;
		*frame_height_pt = 144;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}

	else if ( strcmp(size,"sif") == 0 )
	{
		*frame_width_pt = 320;
		*frame_height_pt = 240;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}

	else if ( strcmp(size,"cif") == 0 )
	{
		*frame_width_pt = 352;
		*frame_height_pt = 288;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}

	else if ( strcmp(size,"vga") == 0 )
	{
		*frame_width_pt = 640;
		*frame_height_pt = 480;
		printf("Capture window size = [%d,%d]\n",*frame_width_pt, *frame_height_pt);
	}
	
	else {
		printf(" Unrecognized frame size");
	}
		
	return 0;
}


//=============================================================================

int ThresholdFile( char *fname, BWImage **bwIm, int color){
	
	FILE* file;
	int width, height;
	int i,num_read;
	unsigned char r,g,b;    /* RGB values of each pixel */   
	float sumPixel;     
	int Bnorm = 100;  	/* Threshold to avoid noise amplification in dark regions */
	
	// Open the file
	file = fopen(fname, "r");
	if (file == NULL)
	{
		printf("ThresholdFile: failed to open file\n");
		return -1;
	}
	
	ProcessHeaders(file, &width, &height);
	CreateBWImage(bwIm, height+1, width);     
	
	//Add one line of black pixels for blob extraction step
	for (i=0; i<(*bwIm)->cols;i++){
		(*bwIm)->data[i] = 0;
	}
	
	//Read data from file
	for(i=0; i<width*height; i++){
		
		//Read one value at a time
		num_read=0;      
		num_read+= fscanf(file,"%c",&r); /*RGB values*/
		num_read+= fscanf(file,"%c",&g);
		num_read+= fscanf(file,"%c",&b);
		
		if (num_read != 3)
		{
			printf("CreateBWImageFromFile: failed to read RGB values from file : numread = %d\n",num_read);
			fclose(file);
			return -1;
		} 
		
		//Normalize RGB values to amplify the strength of the dominant channel 
		
		sumPixel = r+g+b; 
		if(sumPixel >= Bnorm){               
			r =  (255/(double)sumPixel) * r;
			g =  (255/(double)sumPixel) * g;
			b =  (255/(double)sumPixel) * b;
		}
		
		switch (color) {
			case  RED: 
				//printf("Red component of pixel %d after normalization : %d\n",i,r);
				//getchar();
				if (r>=R_MIN){
					(*bwIm)->data[i+width] = 1; //light the pixel 
					//printf("Pixel %d acceso\n",i);
				}  
				else
					(*bwIm)->data[i+width] = 0; //turn the pixel off
				break;   
				/*case  GREEN:  //green
					if (g>=G_MIN){
						(*bwIm)->data[i+width] = 1; //light the pixel
					}
				else
					(*bwIm)->data[i+width] = 0; //turn the pixel off
				break;
				case  BLUE:  //blue
					if (b>=B_MIN){
						(*bwIm)->data[i+width] = 1; //light the pixel
					} 
				else
					(*bwIm)->data[i+width] = 0; //turn the pixel off
				break;*/
			default :
				printf("Unrecognized color\n");
			    getchar();
				break;
		} 
		
	}	
	
	fclose(file);
	
	return 0;
}

//=============================================================================


int ProcessHeaders(FILE *file, int *width, int *height) {
	
  int num_read;
  char line[MAX_LINE_LEN];

  //Ignore first two lines
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read from file\n");
    fclose(file);
    return -1;
  }

  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read from file\n");
    fclose(file);
    return -1;
  }

  //Read number of rows and columns
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read from file\n");
    fclose(file);
    return -1;
  }

  num_read = sscanf(line,"%d %d",width,height);
  if (num_read != 2)
  {
    printf("ProcessHeaders: failed to read height and width from file\n");
    fclose(file);
    return -1;
  }
	
  //Read max value line
  fgets(line,MAX_LINE_LEN,file);
  if (line == NULL)
  {
    printf("ProcessHeaders: failed to read max value\n");
    fclose(file);
    return -1;
  }
	
	return 0;
}
	
//=============================================================================


int LoadBWImageFromFile( char *fname, BWImage **bwIm){
	
	FILE* file;
	int width, height;
	int i,num_read;
	unsigned char r,g,b;    /* RGB values of each pixel */
	
	// Open the file
	file = fopen(fname, "r");
	if (file == NULL)
	{
		printf("LoadBWImage: failed to open file\n");
		return -1;
	}
	
	ProcessHeaders(file, &width, &height);
	CreateBWImage(bwIm, height, width);     
	
	//EDL no need for this because the image is already the result of a thresholding
	//Add one line of black pixels for blob extraction step
	//for (i=0; i<(*bwIm)->cols;i++){
	//	(*bwIm)->data[i] = 0;
	//}
	
	//Read data from file
	for(i=0; i<width*height; i++){
		
		//Read one value at a time
		num_read=0;      
		num_read+= fscanf(file,"%c",&r); /*RGB values*/
		num_read+= fscanf(file,"%c",&g);
		num_read+= fscanf(file,"%c",&b);
		
		if (num_read != 3)
		{
			printf("LoadBWImageFromFile: failed to read RGB values from file : numread = %d\n",num_read);
			fclose(file);
			return -1;
		} 
		
		if(r==255 && g==255 && b==255)  //white pixel;
			
			(*bwIm)->data[i+width] = 1; //light the pixel 
		
		if(r==0 && g==0 && b==0)  //black pixel;
			
			(*bwIm)->data[i+width] = 0; //light the pixel 
		
	}
	
	fclose(file);
	
	return 0;
}
	

//=============================================================================
