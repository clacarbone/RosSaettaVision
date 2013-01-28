/***************************************************************************
 * File:   crdtransf.c
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/
#include "crdtransf.h"

//****************************************************************************
void set_homogeneous_matrix_gl2lo(float angle, matrix_p o_gl2lo, matrix_p a_gl2lo){
    init_matrix(a_gl2lo,DIM+1,DIM+1);
    
    a_gl2lo->val[0][0]=cos(angle);
    a_gl2lo->val[0][1]=-sin(angle);
    a_gl2lo->val[0][2]=o_gl2lo->val[0][0];
    a_gl2lo->val[1][0]=sin(angle);
    a_gl2lo->val[1][1]=cos(angle);
    a_gl2lo->val[1][2]=o_gl2lo->val[0][1];
    a_gl2lo->val[2][0]=0;
    a_gl2lo->val[2][1]=0;
    a_gl2lo->val[2][2]=1;          
}
//****************************************************************************

void coordinate_tranformation_gl2lo(matrix_p a_gl2lo, matrix_p p_gl, matrix_p p_lo){
    matrix_p temp1, temp2, temp3;

    new_matrix(temp1);
    new_matrix(temp2);
    new_matrix(temp3);
    init_matrix(temp1,1,1);

    ones(temp1);
    append_by_row(p_gl,temp1,temp2);
    clear_matrix(temp1);
    inverse(a_gl2lo,temp1);
    product_rows_by_cols(temp1,temp2,temp3);
    delete_row(temp3,DIM,p_lo);
    free_matrix(temp1);
    free_matrix(temp2);
    free_matrix(temp3);
}
//****************************************************************************

void coordinate_trasformation_lo2gl(matrix_p a_gl2lo, matrix_p p_lo, matrix_p p_gl){
    product_rows_by_cols(a_gl2lo,p_gl,p_lo);
}
//****************************************************************************