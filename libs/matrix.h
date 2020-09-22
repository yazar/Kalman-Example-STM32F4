
#ifndef MATRIX_H_
#define MATRIX_H_

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdio.h>
#include <math.h>

void multiplyMatrices22(float first[2][2],float second[2][2],float result[2][2]);
float determinant22(float [][2], float k);
void inverse22(float num[2][2],float res[2][2] , float r);
void transpose22(float mat1[2][2], float mat2[2][2]);
void multiplyMatrices21(float first[2][2],float second[2][1],float result[2][1]);
void multiplyMatrices211(float first[2][1],float second[1][1],float result[2][1]);
void addMatrices22(float first[2][2],float second[2][2],float result[2][2]);
void subMatrices22(float first[2][2],float second[2][2],float result[2][2]);
void addMatrices21(float first[2][1],float second[2][1],float result[2][1]);
void subMatrices21(float first[2][1],float second[2][1],float result[2][1]);


#ifdef __cplusplus
  }
#endif
#endif  