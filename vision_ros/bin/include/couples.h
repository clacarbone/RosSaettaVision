#ifndef _COUPLES_H_
#define _COUPLES_H_


#ifdef __cplusplus
extern "C" {
#endif

#define _ISOC99_SOURCE      
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_math.h>
#include "coordinates.h"
#include "globals.h"
#include "lists.h"
#include "matrix.h"

typedef struct {
	float x;
	float y;
}	marker;

typedef struct {
	marker *markers;
	int numMarker;
}	markerList;

markerList *m1List=NULL;	//lista rossi
markerList *m2List=NULL;	//lista verdi

//coppie di marker
typedef struct
{
  float x1;
  float y1;
  float x2;
  float y2;
  matrix_p coord;  
 } couple;

typedef struct
{
  couple *cpl;    	//pointer to first element in list
  int numCpl;
} coupleList;

coupleList *cplList;

/**
 * This function aims to create the couples of red and green markers. The number of couples is equal to the minimum between the number
 * of red and green markers.
 * */
void create_couples();

#ifdef __cplusplus
}
#endif
  
#endif
