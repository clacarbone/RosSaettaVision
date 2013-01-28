//==========================================================================
//
//  Project:        FoxVision: Vision system for mobile robots equipped with 
//					FOX board Netus G20     
//
//  Module:         Frame implementation
//
//  Description:    Each frame captured by the FRAMEGRABBER returns a FRAME
//                  (defined here).  It contains the raw frame data, as well
//                  as information about the frame's size and format.\llo
//
//  Author:          Enrico Di Lello
//
//--------------------------------------------------------------------------
//
//  FoxVision: Vision system for mobile robots
//  Copyright (c) 2009 Enrico Di Lello
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
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

#include "frame.h"
#include "simple_image.h"

#include <stdio.h>
#include <stdlib.h>

#include <linux/videodev.h>

//==============================================================================

int CreateFrame( FRAME **fr , int width, int height, int format )
{
	 *fr = (FRAME*) malloc ( sizeof( FRAME ) );
	
	if ( *fr == NULL)
	{
		printf("CreateFrame: Cannot create frame (out of memory?)...\n");
		return -1;
	}
    
	int size = 0;

    (*fr)->width = width;
    (*fr)->height = height;
    (*fr)->format = format;

	// Compute buffer size in bytes according to the specified format
    size = FrameGetSize( *fr );

    (*fr)->data = malloc( size );

	if ( (*fr)->data == NULL)
	{
		printf("CreateFrame: Cannot create data field (out of memory?)...\n");
		return -1;
	}
    
    return 0;
}

//==============================================================================


void ReleaseFrame( FRAME **fr )
{
    free( (*fr)->data );
    free( (*fr) );
}


//==============================================================================


int FrameGetSize( FRAME* fr )
{
    int pixels = fr->width * fr->height;
    int size = 0;

    switch ( fr->format )
    {
        case V4L2_PIX_FMT_BGR24:
            // 3 bytes per pixel
            size = pixels * 3;
            break;

		case V4L2_PIX_FMT_YUYV:
		    // 2 bytes per pixel
			size = pixels * 2;
			break;
		case VIDEO_PALETTE_YUV420P:
			// 1.5 bytes per pixel
			size = pixels *1.5;
			break;
			
        default:
            // Unsupported!
            fprintf( stderr, "FrameGetSize(): Unexpected type!\n" );
            size = -1;
    }

    return size;
}


//==============================================================================

int SaveFrame( FRAME* fr, const char* filename )
{
	int i = 0;
	FILE* fp = fopen( filename, "w" );
	unsigned char *yuyv;
	int x,z;

	if ( fp == NULL )
	{
		perror( "frame_save(): opening file for writing" );
		return -1;
	}
	
	// Write PNM header
	fprintf( fp, "P6\n" );
	fprintf( fp, "# Generated by the one who knows how stuff work!\n" );
	fprintf( fp, "%d %d\n", fr->width, fr->height );
	
	// Max val
	fprintf( fp, "255\n" );
	
	yuyv = fr->data;

	for ( i=0; i < fr->height; i++) {
 		z = 0;

    		for (x = 0; x < fr->width; x++) {
      			int r, g, b;
      			int y, u, v;

      			if (!z)
        			y = yuyv[0] << 8;
      			else
        			y = yuyv[2] << 8;
      			u = yuyv[1] - 128;
      			v = yuyv[3] - 128;

      			r = (y + (359 * v)) >> 8;
      			g = (y - (88 * u) - (183 * v)) >> 8;
     	 		b = (y + (454 * u)) >> 8;

      			r = (r > 255) ? 255 : ((r < 0) ? 0 : r);
      			g = (g > 255) ? 255 : ((g < 0) ? 0 : g);
      			b = (b > 255) ? 255 : ((b < 0) ? 0 : b);

      			if (z++) {
        			z = 0;
        			yuyv += 4;
      			}
		fprintf(fp,"%c%c%c", r,g,b);
	
    		}
	}
	fclose(fp);
return 0;
}
//==============================================================================

