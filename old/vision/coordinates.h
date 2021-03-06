//==========================================================================
//
//  Project:        guarDIAn: Vision system for mobile robots equipped with 
//					FOX board Netus G20
//
//  Module:         Distance computation from pixel indexes
//
//  Description:    Definitions of a set of function to manage pixel to 
//					XY coordinates correspondence.
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
//==========================================================================

//#ifndef distance_h_DEFINED
//#define distance_h_DEFINED

#ifndef coordinates_h_DEFINED
#define coordinates_h_DEFINED

//================
//   Includes
//================

#include "globals.h"
#include "lists.h"
#include <math.h>
#include "matrix.h"
//===============
//   Defines
//===============
#define GIALLO "\033[1;33m"
#define ROSSO "\033[31m"
#define NORMAL "\033[0m"

#define PRINTW(format, args...)  printf(GIALLO format NORMAL "\n", ##args);
#define PRINTE(format, args...)  printf(ROSSO format NORMAL "\n", ##args);





//------------------------------------------------------------------------------
///
/// \brief Contains webcam information of: color blob, cartesian coordinates and polar coordinates
/// 
//------------------------------------------------------------------------------

typedef struct
{
  int color;
  float centroidX;              // X coordinate in the of the blob centroid in the floor reference frame 
  float centroidY;		// Y coordinate in the of the blob centroid in the floor reference frame
  float centroid_distance;      // estimated distance
  float centroid_angle;         // estimated deviation angle  
} colorBlob;

typedef struct
{
  colorBlob *blobs;    	//pointer to first element in list
  int numBlob;          //numero di blob
} colorBlobList;

colorBlobList *colorBList;

int contatore22;//=0;	//da eliminare serve solo per simulare acquisizione 

/*
typedef struct
{
	int id;
	float xm1;
	float ym1;
	float xm2;
	float ym2;
	float x;
	float y;
	float distance;
	float orientation;
} robot;

typedef struct
{
  robot *robots;    	//pointer to first element in list
  int numRob;          //numero di blob
} robotList;

robotList *robList;
 


typedef struct{
	int color;
	float robot_X;
	float robot_Y;
	float robot_distance;
	float robot_orientation;	
}	colorRobot;

typedef struct
{
	colorRobot *robots;
	int numRobot;
}	colorRobotList;

colorRobotList *colorRList;

typedef struct{
	//int color;
	float markerX;
	float markerY;
	float marker_distance;
}	commonMarker;

typedef struct
{
	commonMarker *markers;
	int numMarker;
}	commonMarkerList;

commonMarkerList *commonMList;
*/


//------------------------------------------------------------------------------
///
/// \brief Contains mapping information between camera pixel and XY coordinates
/// 
///   This structure contains two array which contain a X and Y coordinates
///   relative to a fixed frame. This information are computed previously
///   during camera calibration
//------------------------------------------------------------------------------

typedef struct
{
  int rows;
  int cols;
  float *x_coord;
  float *y_coord;
  
} PixelMap;

PixelMap *pixelMap;		// map between pixel coordinates and XY coordinates 


int numRobots; //da togliere

//==============================
//    Function prototypes
//==============================

//------------------------------------------------------------------------------
///
///	\brief			Read param from file
///
//------------------------------------------------------------------------------
void globals_init();

//------------------------------------------------------------------------------
///
///	\brief			Main PixelMap initialization function
///
///	\param map_file		Nome file di Input
/// 
//------------------------------------------------------------------------------

int InitPixelMap(char *map_file);

//------------------------------------------------------------------------------
///
///	\brief			Main pixel to coordinates mapping  function
/// 
//------------------------------------------------------------------------------

int EvaluateCoordinates();

//------------------------------------------------------------------------------
///
/// \brief Create a new PixelMap 
///
/// \param  pixelMap   PixelMap structure pointer
/// \param  rows       image rows  (pixels)
/// \param  cols       image columns (pixels)
///
//------------------------------------------------------------------------------

int CreatePixelMap(PixelMap **pixelMap, int rows, int cols);

//------------------------------------------------------------------------------
///
/// \brief read data from a file and use ti to initialize a PixelMap structure
///
/// \param  map_file   text file containing pixel to coordinate mapping
/// \param  pixelMap   pointer to a pixelMap
///
///
//------------------------------------------------------------------------------

int FillPixelMap(char *map_file, PixelMap *pixelMap);

//------------------------------------------------------------------------------
///
/// Print the PixelMap elements
///
//------------------------------------------------------------------------------

int PrintPixelMap(PixelMap *pixelMap);	



//------------------------------------------------------------------------------
///
/// Compute the x,y coordinates respect to a fixed reference system
/// Coordinates for every pixel are stored in a file
//------------------------------------------------------------------------------

int GetCoordinates(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit,float *packet_data);

//------------------------------------------------------------------------------
///
/// Compute the x,y coordinates respect to a fixed reference system
/// Coordinates for every pixel are stored in a file
//------------------------------------------------------------------------------

int GetCoordinatesEvo(ComponentsList *compList, PixelMap *pixelMap, FILE *log_file, int transmit,float *packet_data);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
///
/// Calcola le coordinate (sia polari che cartesiane) delle componenti colorate e le memorizza in una struttura 
//------------------------------------------------------------------------------

//int GetColorBlob(colorBlobList **colorBList, colorRobotList **colorRList, colorMarkerList **colorMList, ComponentsList *compList, PixelMap *pixelMap, FILE *log_file);

int GetColorBlob(colorBlobList **colorBList, ComponentsList *compList, PixelMap *pixelMap, FILE *log_file);

//------------------------------------------------------------------------------
/// \brief read x,y estimed coordinates and return estimated distance from fixed point
/// 
/// \param x,y centroid float coordinates fixed 0,0 reference system
/// 
//------------------------------------------------------------------------------

float GetDistCentroidBlob(float x_cent_blob, float y_cent_blob);

//------------------------------------------------------------------------------
///
/// \brief read x,y estimed coordinates and return estimated deviation angle 
/// 
/// \param x,y centroid float coordinates fixed 0,0 reference system
/// 
//------------------------------------------------------------------------------

float GetAngleCentroidBlob(float x_cent_blob, float y_cent_blob);

/*
//------------------------------------------------------------------------------
///
/// \return a info webcam blob estimation structure 
/// 
//------------------------------------------------------------------------------

void GetStructureWebCamBlob(WebCamBlobEstimation** wstr,int *n);
*/

#endif

