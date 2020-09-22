#include "matrix.h"




void multiplyMatrices22(float first[2][2],float second[2][2],float result[2][2]) {
	
	int r1 = 2,c1=2, r2=2,c2=2;

   // Initializing elements of matrix mult to 0.
   for (int i = 0; i < r1; ++i) {
      for (int j = 0; j < c2; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < r1; ++i) {
      for (int j = 0; j < c2; ++j) {
         for (int k = 0; k < c1; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

void multiplyMatrices21(float first[2][2],float second[2][1],float result[2][1]) {
	
	int r1 = 2,c1=2, r2=2,c2=1;

   // Initializing elements of matrix mult to 0.
   for (int i = 0; i < r1; ++i) {
      for (int j = 0; j < c2; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < r1; ++i) {
      for (int j = 0; j < c2; ++j) {
         for (int k = 0; k < c1; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

void multiplyMatrices211(float first[2][1],float second[1][1],float result[2][1]) {
	
	int r1 = 2,c1=1, r2=1,c2=1;

   // Initializing elements of matrix mult to 0.
   for (int i = 0; i < r1; ++i) {
      for (int j = 0; j < c2; ++j) {
         result[i][j] = 0;
      }
   }

   // Multiplying first and second matrices and storing it in result
   for (int i = 0; i < r1; ++i) {
      for (int j = 0; j < c2; ++j) {
         for (int k = 0; k < c1; ++k) {
            result[i][j] += first[i][k] * second[k][j];
         }
      }
   }
}

float determinant22(float a[2][2], float k)
{
  float s = 1, det = 0, b[2][2];
  int i, j, m, n, c;
  if (k == 1)
    {
     return (a[0][0]);
    }
  else
    {
     det = 0;
     for (c = 0; c < k; c++)
       {
        m = 0;
        n = 0;
        for (i = 0;i < k; i++)
          {
            for (j = 0 ;j < k; j++)
              {
                b[i][j] = 0;
                if (i != 0 && j != c)
                 {
                   b[m][n] = a[i][j];
                   if (n < (k - 2))
                    n++;
                   else
                    {
                     n = 0;
                     m++;
                     }
                   }
               }
             }
          det = det + s * (a[0][c] * determinant22(b, k - 1));
          s = -1 * s;
          }
    }
 
    return (det);
}


void inverse22(float num[2][2],float res[2][2] , float r)
{
	float fac[2][2];
	int f = 2;
	float b[2][2];
 for (int q = 0;q < f; q++)
 {
   for (int p = 0;p < f; p++)
    {
     int m = 0;
     int n = 0;
     for (int i = 0;i < f; i++)
     {
       for (int j = 0;j < f; j++)
        {
          if (i != q && j != p)
          {
            b[m][n] = num[i][j];
            if (n < (f - 2))
             n++;
            else
             {
               n = 0;
               m++;
               }
            }
        }
      }
      fac[q][p] = pow(-1, q + p) * determinant22(b, f - 1);
    }
  }
  int i, j;
  float inverse[2][2], d;
 
  for (i = 0;i < r; i++)
    {
     for (j = 0;j < r; j++)
       {
         b[i][j] = fac[j][i];
        }
    }
  d = determinant22(num, r);
  for (i = 0;i < r; i++)
    {
     for (j = 0;j < r; j++)
       {
        inverse[i][j] = b[i][j] / d;
        }
    }
	res[0][0] = inverse[0][0];
	res[0][1] = inverse[0][1];
	res[1][0] = inverse[1][0];
	res[1][1] = inverse[1][1];
 
}

void transpose22(float mat1[2][2], float mat2[2][2])
{
	for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            mat2[j][i] = mat1[i][j];
        }
}

void addMatrices22(float first[2][2],float second[2][2],float result[2][2]){
	int r = 2, c = 2;
	for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) {
            result[i][j] = first[i][j] + second[i][j];
        }
}

void subMatrices22(float first[2][2],float second[2][2],float result[2][2]){
	int r = 2, c = 2;
	for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) {
            result[i][j] = first[i][j] - second[i][j];
        }
}

void addMatrices21(float first[2][1],float second[2][1],float result[2][1]){
	int r = 2, c = 1;
	for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) {
            result[i][j] = first[i][j] + second[i][j];
        }
}

void subMatrices21(float first[2][1],float second[2][1],float result[2][1]){
	int r = 2, c = 1;
	for (int i = 0; i < r; ++i)
        for (int j = 0; j < c; ++j) {
            result[i][j] = first[i][j] - second[i][j];
        }
}