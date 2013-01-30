/***************************************************************************
 * File:   matrix.c
 * Author: Daniela Carboni
 *
 * Created on novembre 2010
 ****************************************************************************/

#include "matrix.h"
//****************************************************************************

void new_matrix(matrix_p* a) {
    *a = malloc(sizeof(matrix_t));
}
//****************************************************************************

void init_matrix(matrix_p a, int row, int col) {
    int i;
    a->row = row;
    a->col = col;
    a->val = (float**) malloc(row * (sizeof(float*)));
    for (i = 0; i < row; i++)
        a->val[i] = (float*) malloc(col * (sizeof(float)));
}
//****************************************************************************

void free_matrix(matrix_p a){
    int i;
    for(i=0;i<(a)->row;i++)
        free(a->val[i]);
    free(a->val);
    free(a);
}
//****************************************************************************

void clear_matrix(matrix_p a) {
    int i;
    for (i = 0; i < (a)->row; i++)
        free(a->val[i]);
    free(a->val);
}
//****************************************************************************

void zeros(matrix_p a){
    int i,j;
    for(i=0;i<a->row;i++)
        for(j=0;j<a->col;j++)
            a->val[i][j]=0;
}
//****************************************************************************

void ones(matrix_p a){
    int i,j;
    for(i=0;i<a->row;i++)
        for(j=0;j<a->col;j++)
            a->val[i][j]=1;
}
//****************************************************************************

void cnst(float valc, matrix_p a){
	int i,j;
	for(i=0;i<a->row;i++)
		for(j=0;j<a->col;j++)
			a->val[i][j]=valc;
}
//****************************************************************************
void eye(matrix_p a){
    int i,j;
    for(i=0;i<a->row;i++)
        for(j=0;j<a->col;j++)
            if(i==j)
                a->val[i][j]=1;
            else
                a->val[i][j]=0;
}
//****************************************************************************

void print_matrix(matrix_p a) {
    int i, j;
    printf("Matrix %dx%d:\n\t", a->row, a->col);
    for (i = 0; i < a->row; i++) {
        for (j = 0; j < a->col; j++)
            printf("%lf ", a->val[i][j]);
        printf("\n\t");
    }
}
//****************************************************************************
fprint_matrix(FILE *ff, matrix_p mat){
	int i,j;
	for(i=0;i<mat->row;i++){
		for(j=0;j<mat->col;j++)
			fprintf(ff,"%f\t",mat->val[i][j]);
		fprintf(ff,"\n");
	}
}
//****************************************************************************
fprint_vector_inrow(FILE *ff,matrix_p mat){
	if(mat->row>1 && mat->col>1){
		printf("ERRORE: non è un vettore\n");
			return;
	}	
	int i;
	if(mat->row>1){	//se è un vettore colonna
		for(i=0;i<mat->row;i++)
			fprintf(ff,"%f\t",mat->val[i][0]);
	}
	else {
		for(i=0;i<mat->col;i++)
			fprintf(ff,"%f\t",mat->val[0][i]);
	}
	fprintf(ff,"\n");
}

//****************************************************************************
void sum(matrix_p op1, matrix_p op2, matrix_p res) {
    if (!sameSize(op1,op2))
        printf("Error: matrix dimension must agree");
    else {
        init_matrix(res, op1->row, op1->col);
        int i, j;
        for (i = 0; i < op1->row; i++)
            for (j = 0; j < op1->col; j++)
                res->val[i][j] = op1->val[i][j] + op2->val[i][j];
    }
}
//****************************************************************************

void sub(matrix_p op1, matrix_p op2, matrix_p res) {
    if (!sameSize(op1,op2))
        printf("Error: matrix dimension must agree");
    else {
        init_matrix(res, op1->row, op1->col);
        int i, j;
        for (i = 0; i < op1->row; i++)
            for (j = 0; j < op1->col; j++)
                res->val[i][j] = op1->val[i][j] - op2->val[i][j];
    }
}
//****************************************************************************

void product_rows_by_cols(matrix_p op1, matrix_p op2,matrix_p res){
    if(op1->col!=op2->row){
        printf("Error: inner matrix dimension must agree\n");
        printf("op1: %d\top2: %d\n",op1->col,op2->row);
        return;
    }
    init_matrix(res,op1->row,op2->col);
    zeros(res);
    int i,j,k;

    for(i=0;i<res->row;i++)
        for(j=0;j<res->col;j++){
            k=0;
            while(k<op1->col){
            res->val[i][j]+=op1->val[i][k]*op2->val[k][j];
            k++;
            }
        }
}
//****************************************************************************

void product_scalar_by_matrix(float scalar,matrix_p mat,matrix_p res){
    init_matrix(res,mat->row,mat->col);
    int i,j;
    for(i=0;i<res->row;i++)
        for(j=0;j<res->col;j++)
            res->val[i][j]=scalar*mat->val[i][j];
}
//****************************************************************************

void submatrix(matrix_p mat, matrix_p submat, int row, int col) {
    int i, j, r, c;
    init_matrix(submat, mat->row-1,mat->col-1);
    r = c = 0;
    for (i = 0; i < mat->row; i++)
        for (j = 0; j < mat->col; j++)
            if (i != row && j != col) {
                submat->val[r][c] = mat->val[i][j];
                if (c != submat->col - 1)
                    c++;
                else {
                    c = 0;
                    r++;
                }
            }
}
//****************************************************************************

int sign(int i, int j) {
    if ((i + j) % 2)
        return -1;
    return 1;
}
//****************************************************************************

float det(matrix_p mat) {
    float d = 0;
    int j;

    if (!isSquare(mat))
        printf("Errore: matrix must be square");
    else
        if (mat->row == 1)
        d = mat->val[0][0];
    else {
        for (j = 0; j < mat->col; j++) {
            matrix_p temp;
            new_matrix(&temp);
            submatrix(mat,temp,0,j);
            d += sign(0, j)*mat->val[0][j]*det(temp);
            free_matrix(temp);
        }
    }
    return d;
}
//****************************************************************************

void transpose(matrix_p mat, matrix_p transmat) {
    init_matrix(transmat,mat->col,mat->row);
    int i, j;
    for (i = 0; i < transmat->row; i++)
        for (j = 0; j < transmat->col; j++)
            transmat->val[i][j] = mat->val[j][i];
}
//****************************************************************************

int isIdentity(matrix_p mat) {
    if (mat->row != mat->col)
        return 0;
    int i, j;
    for (i = 0; i < mat->row; i++)
        for (j = 0; j < mat->col; j++)
            if (i == j) {
                if (mat->val[i][j] != 1)
                    return 0;
            } else {
                if (mat->val[i][j] != 0)
                    return 0;
            }
    return 1;
}
//****************************************************************************

int isSquare(matrix_p mat){
    if(mat->row==mat->col)
        return 1;
    return 0;
}
//****************************************************************************

int isInvertible(matrix_p mat){
    if(det(mat)==0)
        return 0;
    return 1;
}
//****************************************************************************

int sameSize(matrix_p mat1, matrix_p mat2){
    if(mat1->row==mat2->row && mat1->col==mat2->col)
        return 1;
    return 0;
}
//****************************************************************************

void inverse(matrix_p mat,matrix_p invmat) {
	//print_matrix(mat);
	
	float temp;
    int i, j, k, n;
	if (!isSquare(mat)){
        printf("Error: matrix must be square\n");
        return;
    }
    if (det(mat)==0){ //!isInvertible(mat)){
        printf("Error: matrix is singular\n ");
        return;
    }
	n = mat->row;
    init_matrix(invmat,n,n);
    matrix_p app;
    new_matrix(&app);
    init_matrix(app,n,n);
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            app->val[i][j] = mat->val[i][j];

    //all'inizio la matrice inversa è la matrice identità da affiancare alla matrice mat
    init_matrix(invmat,n,n);
    eye(invmat);

    
    //Applico algoritmo Gauss Jordan
    while (!isIdentity(app)) {
        for (k = 0; k < n; k++) {
            temp = app->val[k][k];
            if (temp != 0)
                for (j = 0; j < n; j++) {
                    app->val[k][j] /= temp;
                    invmat->val[k][j] /= temp;
                } else {
                i = 0;
                while (app->val[i][k] == 0 && i < n)
                    i++;
                for (j = 0; j < n; j++) {
                    app->val[k][j] += app->val[i][j] / app->val[i][k];
                    invmat->val[k][j] += invmat->val[i][j] / app->val[i][k];
                }
            }
        }
        
        for (i = 0; i < n; i++)
            for (j = 0; j < n; j++) {
                if (i != j && app->val[i][j] != 0) {
                    temp = app->val[i][j];
                    for (k = 0; k < n; k++) {
                        app->val[i][k] -= app->val[j][k] * temp;
                        invmat->val[i][k] -= invmat->val[j][k] * temp;
                    }
                }
            }
    }
    free_matrix(app);
}
//****************************************************************************
void inverse3x3(matrix_p mat,matrix_p invmat) {
	if (!isSquare(mat)){
        printf("Error: matrix must be square");
        return;
    }
    if (!isInvertible(mat)){
        printf("Error: matrix is singular ");
        return;
    }	
	float d=det(mat);
	int n = mat->row;
	printf("Determinante: %f\n",d);
    init_matrix(invmat,n,n);
	invmat->val[0][0]=((mat->val[1][1]*mat->val[2][2])-(mat->val[1][2]*mat->val[2][1]))/d;
	invmat->val[0][1]=((mat->val[0][2]*mat->val[2][1])-(mat->val[0][1]*mat->val[2][2]))/d;
	invmat->val[0][2]=((mat->val[0][1]*mat->val[1][2])-(mat->val[0][2]*mat->val[1][1]))/d;
	invmat->val[1][0]=((mat->val[1][2]*mat->val[2][0])-(mat->val[1][0]*mat->val[2][2]))/d;
	invmat->val[1][1]=((mat->val[0][0]*mat->val[2][2])-(mat->val[0][2]*mat->val[2][0]))/d;
	invmat->val[1][2]=((mat->val[0][2]*mat->val[1][0])-(mat->val[0][0]*mat->val[1][2]))/d;
	invmat->val[2][0]=((mat->val[0][1]*mat->val[2][1])-(mat->val[1][1]*mat->val[2][0]))/d;
	invmat->val[2][1]=((mat->val[0][1]*mat->val[2][0])-(mat->val[0][0]*mat->val[2][1]))/d;
	invmat->val[2][2]=((mat->val[0][0]*mat->val[1][1])-(mat->val[0][1]*mat->val[1][0]))/d;
	print_matrix(invmat);
	if ( isnan(d)!=0) {
		print_matrix(mat);

		exit(0);
	}

}

//****************************************************************************
void read_from_file(matrix_p mat, char *path){
    int i,j,matrix_full;

    FILE *matrix_file;
    matrix_file=fopen(path,"r");
    if(matrix_file==NULL) {
	printf("Error on file opening\n");
    }
    matrix_full=0;
    i=j=0;
    while (fscanf(matrix_file, "%f\n",mat->val[i][j]) != EOF) { //per riga
        if (j == mat->col-1) {
            j = 0;
            i++;
            if (i == mat->row)
                matrix_full=1;
        } else
            j++;
    }
    fclose(matrix_file);
}
//****************************************************************************

void copy(matrix_p mat1,matrix_p mat2){
    if(!sameSize(mat1,mat2)){
        printf("Error: matrix dimension must agree");
        printf("\n\n mat1: %d x %d\nmat2: %d x %d\n",mat1->row,mat1->col,mat2->row,mat2->col);
        return;
    }
    int i,j;
    for(i=0;i<mat1->row;i++)
        for(j=0;j<mat1->col;j++)
            mat2->val[i][j]=mat1->val[i][j];
}
//****************************************************************************

void append_by_row(matrix_p mat1, matrix_p mat2,matrix_p new_mat){
    init_matrix(new_mat,mat1->row+mat2->row,mat1->col);
    if(mat1->col!=mat2->col){
        printf("Error: column dimension must agree");
        return;
    }
    copy(mat1,new_mat);
    int i,j;
    for(i=mat1->row;i<new_mat->row;i++)
        for(j=0;j<mat1->col;j++)
            new_mat->val[i][j]=mat2->val[i-mat1->row][j];
}
//****************************************************************************

void append_by_col(matrix_p mat1, matrix_p mat2,matrix_p new_mat){
    init_matrix(new_mat,mat1->row,mat1->col+mat2->col);
    if(mat1->row!=mat2->col){
        printf("Error: row dimension must agree");
        return;
    }
    copy(mat1,new_mat);
    int i,j;
    for(i=0;i<mat1->row;i++)
        for(j=mat1->col;j<new_mat->col;j++)
            new_mat->val[i][j]=mat2->val[i][j-mat1->col];
}
//****************************************************************************

void delete_row(matrix_p mat1,int row,matrix_p mat2){
    if(row>=mat1->row){
        printf("Error: index out of bound");
        return;
    }
    init_matrix(mat2,mat1->row-1,mat1->col);

    int i,j,r;
    r=0;
    for(i=0;i<mat1->row;i++){
        if(i!=row){
            for(j=0;j<mat1->col;j++)
                mat2->val[r][j]=mat1->val[i][j];

            r++;
        }
    }
}
//****************************************************************************

void delete_col(matrix_p mat1, int col, matrix_p mat2){
    if(col>=mat1->col){
        printf("Error: index out of bound");
        return;
    }
    init_matrix(mat2,mat1->row,mat1->col-1);
    int i,j,c;
    for(i=0;i<mat1->row;i++){
        c=0;
        for(j=0;j<mat1->col;j++)
            if(j!=col){
                mat2->val[i][c]=mat1->val[i][j];
                c++;
            }
        }
}
//****************************************************************************

float distance(matrix_p m1, matrix_p m2){
	if(!sameSize(m1,m2)){
		printf("Matrix dimension must agree");			
	}
	if(m1->row>3 && m1->col>1 && m1->row!=1){
			return fabs(sqrt(pow((m1->val[0][0]-m2->val[0][0]),2)+pow((m1->val[1][0]-m2->val[1][0]),2)));
	}
	if(m1->row>1 && m1->col>3 && m2->row!=1){
			return fabs(sqrt(pow((m1->val[0][0]-m2->val[0][0]),2)+pow((m1->val[0][1]-m2->val[0][1]),2)));
	}
	return -1;	
}
