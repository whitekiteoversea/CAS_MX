#include "luenberger_observer.h"

matrix_t *x_prev = NULL;

void InitLuembergerMatrices(luenberger_matrices_t *self)
{
  x_prev = (matrix_t*)malloc(sizeof(matrix_t));
	
	self->A = (matrix_t*)malloc(sizeof(matrix_t));
	self->B = (matrix_t*)malloc(sizeof(matrix_t));
	self->C = (matrix_t*)malloc(sizeof(matrix_t));
	self->L = (matrix_t*)malloc(sizeof(matrix_t));

	// TODO
	// Matrices size as input

	// 暂时不知道系统状态空间表达式，先写一个
	InitMatrix(self->A, 3, 3, 0.0);

	self->A->mat[0][0] = 1.12;
	self->A->mat[0][1] = 0.213;
	self->A->mat[0][2] = -0.335;
	self->A->mat[1][0] = 1.0;
	self->A->mat[2][1] = 1.0;

	InitMatrix(self->B, 3, 1, 0.0);

	self->B->mat[0][0] = 1.0;

	InitMatrix(self->C, 1, 3, 0.0);

	self->C->mat[0][0] = 0.0541;
	self->C->mat[0][1] = 0.1150;
	self->C->mat[0][2] = 0.0050;

	/* L Init*/

	InitMatrix(self->L, 3, 1, 0.0);
	self->L->mat[0][0] = 4.9113;
	self->L->mat[1][0] = -0.4055;
	self->L->mat[2][0] = 9.2845;

	InitMatrix(x_prev, 3, 1, 0.0);
}

int LuenbergerObserver(matrix_t* u, matrix_t* y, matrix_t* x_hat, luenberger_matrices_t* matrices) {

	/*
		LUENBERGER OBSERVER
		X_hat(k+1) = Ax(k) + Bu(k) + L[y(k) - Cx(k)]
	*/
	int ret = 0;
	// TODO
	// Be more memory efficient
	matrix_t res_1, res_2, y_hat, res_3, res_4, res_5, res_6;

	// Ax(k)
	ret = MatrixMultiplication(matrices->A, x_hat, &res_1);
	// Bu(k)
	ret = MatrixMultiplication(matrices->B, u, &res_2);

	// L[y(k) - Cx(k)]
	ret = MatrixMultiplication(matrices->C, x_hat, &y_hat);
	ret = MatrixSubtraction(y, &y_hat, &res_3);
	ret = MatrixMultiplication(matrices->L, &res_3, &res_4);

	ret = MatrixSum(&res_1, &res_2, &res_5);
	ret = MatrixSum(&res_5, &res_4, &res_6);

	CopyMatrixValues(&res_6, x_hat);
	//CopyMatrixValues(x_hat, x_prev);
	
	return ret;
}
