/***************************************************************************
 * File:   crdtransf.h
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/

#ifndef _CRDTRANSF_H
#define	_CRDTRANSF_H
//================
//   Includes
//================
#include "matrix.h"

//====================
//     Defines
//====================
#define    DIM     2

//==============================
//    Function prototypes
//==============================
/*
 * Calcola la matrice di trasformazione omogenea (frame globale-->locale)
 */
void set_homogeneous_matrix_gl2lo(float angle, matrix_p o_gl2lo, matrix_p a_gl2lo);

/*
 * Calcola la trasformazione di coordinate (frame globale-->locale)
 */
void coordinate_tranformation_gl2lo(matrix_p a_gl2lo, matrix_p p_gl, matrix_p p_lo);

/*
 * Calcola la trasformazione di coordinate (frame locale-->globale)
 */
void coordinate_trasformation_lo2gl(matrix_p a_gl2lo, matrix_p p_lo, matrix_p p_gl);
#endif	

