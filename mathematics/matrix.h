/***************************************************************************
 * File:   matrix.h
 * Author: Daniela Carboni
 *
 * Created on November 2010
 ****************************************************************************/
#ifndef _MATRIX_H_
#define _MATRIX_H_
//================
//   Includes
//================
#include <stdio.h>
#include <stdlib.h>
#define _ISOC99_SOURCE
#include <math.h>


#define PRINT_MATRIX
//====================
//     Structures
//====================
typedef struct {
    int row;
    int col;
    float** val;
} matrix_t;

typedef matrix_t* matrix_p;

//==============================
//    Function prototypes
//==============================
/*
 * Crea nuova matrice
 */
void new_matrix(matrix_p* a);

/*
 * Inizializza matrice
 */
void init_matrix(matrix_p a, int row, int col);

/*
 * Distrugge matrice
 */
void free_matrix(matrix_p a);

/*
 * Cancella matrice  
 */
void clear_matrix(matrix_p a);

/*
 * Assegna a tutti gli elementi di a valore 0
 */
void zeros(matrix_p a);

/*
 * Assegna a tutti gli elementi di a valore 1
 */
void ones(matrix_p a);

/*
 *Assegna a tutti gli elementi di a valore val
 */
 void cnst(float valc, matrix_p a);  

/*
 * Trasforma a nella matrice identita'
 */
void eye(matrix_p a);

/*
 * Stampa la matrice a
 */
void print_matrix(matrix_p a);

/*
 * Calcola la somma tra due matrici: res=op1+op2
 */
void sum(matrix_p op1, matrix_p op2, matrix_p res);

/*
 * Calcola la differenza tra due matrici: res=op1+op2
 */
void sub(matrix_p op1, matrix_p op2, matrix_p res);

/*
 * Calcola il prodotto riga per colonna tra due matrici: res=op1*op2
 */
void product_rows_by_cols(matrix_p op1, matrix_p op2,matrix_p res);

/*
 * Calcola il prodotto di uno scalare per una matrice: res=scalar*mat
 */
void product_scalar_by_matrix(float scalar,matrix_p mat,matrix_p res);

/*
 * Calcola la sottomatrice submat ottenuta da mat eliminando la riga row riga e la colonna col
 */
void submatrix(matrix_p mat, matrix_p submat, int row, int col);

/*
 * Calcola il determinante della matrice mat
 */
float det(matrix_p mat);

/*
 *Calcola la trasposta transmat della matrice mat
 */
void transpose(matrix_p mat, matrix_p transmat);

/*
 * Verifica se la matrice mat è la matrice identità
 */
int isIdentity(matrix_p mat);

/*
 * Verifica se la matrice mat è quadrata
 */
int isSquare(matrix_p mat);

/*
 * Verifica se la matrice mat è singolare
 */
int isInvertible(matrix_p mat);

/*
 * Verifica se le matrici mat1 e mat2 hanno la stessa dimensione
 */
int sameSize(matrix_p mat1, matrix_p mat2);

/*
 *Calcola l'inversa della matrice mat
 */
void inverse(matrix_p mat,matrix_p invmat);

/*
 * Legge i valori da file e li memorizza nella matrice
 */
void read_from_file(matrix_p mat, char *path);

/*
 * Copia i valori della matrice mat1 alla matrice mat2
 */
void copy(matrix_p mat1,matrix_p mat2);

/*
 * Concatena le matrici mat1 e mat2 per riga:
 *          mat1
 * new_mat= ----
 *          mat2
 */
void append_by_row(matrix_p mat1, matrix_p mat2, matrix_p new_mat);

/*
 * Concatena le matrici mat1 e mat2 per colonna:
 *
 * new_mat=[mat1|mat2]
 */
void append_by_col(matrix_p mat1, matrix_p mat2, matrix_p new_mat);

/*
 * Crea la matrice mat2 cancellando dalla matrice mat1 la riga row
 */
void delete_row(matrix_p mat1,int row,matrix_p mat2);

/*
 * Crea la matrice mat2 cancellando dalla matrice mat1 la colonna col
 */
void delete_col(matrix_p mat1, int col, matrix_p mat2);

fprint_vector_inrow(FILE *ff,matrix_p mat);

#endif
