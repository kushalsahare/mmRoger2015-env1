// Tue Oct 26 16:48:56 2010  Justus Piater  <justus.piater@uibk.ac.at>

#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cblas.h>
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "student.h"

// #define DEBUG

#ifdef DEBUG
# include <stdio.h>
#endif


#define UNSEEN (-NPIXELS)


// Compute camera matrix P with respect to Roger's base coordinate system:
// P = K R [I | -c]
// x = P X, in homogeneous coordinates
// The return value must be freed with gsl_matrix_free().
static gsl_matrix* getP(const Eye* eye) {
  const double k[] = { FOCAL_LENGTH, NPIXELS / 2.0,
		       0           ,       1        };
  const double r[] = { cos(eye->theta), -sin(eye->theta),
		       sin(eye->theta),  cos(eye->theta)  };
  const double c[] = { 1, 0, -eye->position[Y],    // switched:
		       0, 1, -eye->position[X]  }; // retinas are on y axis
  double kr[4];

  gsl_matrix_const_view K   = gsl_matrix_const_view_array(k, 2, 2);
  gsl_matrix_const_view R   = gsl_matrix_const_view_array(r, 2, 2);
  gsl_matrix_const_view I_c = gsl_matrix_const_view_array(c, 2, 3);
  gsl_matrix_view KR = gsl_matrix_view_array(kr, 2, 2);

  gsl_matrix* P = gsl_matrix_alloc(2, 3);
  gsl_blas_dgemm(CblasNoTrans,
		 CblasNoTrans, 1.0, &K.matrix, &R.matrix, 0.0, &KR.matrix);
  gsl_blas_dgemm(CblasNoTrans,
		 CblasNoTrans, 1.0, &KR.matrix, &I_c.matrix, 0.0, P);

#ifdef DEBUG
  puts("K:");
  gsl_matrix_fprintf(stdout, &K.matrix, "%lf");
  puts("R:");
  gsl_matrix_fprintf(stdout, &R.matrix, "%lf");
  puts("I_c:");
  gsl_matrix_fprintf(stdout, &I_c.matrix, "%lf");
#endif
  return P;
}


// Triangulate world point X (in Roger's egocentric coordinates)
// using the Direct Linear Transform: 
// x = PX and x' = P'X (in homogeneous coordinates)
// Formulate the equations "x collinear PX" and "x' collinear P'X",
// rewrite it as AX = 0, and solve it with SVD.
static void getX(gsl_matrix* P[], int x[], double egoX[]) {
  // x  = [x[0], 1]^T,
  // x' = [x[1], 1]^T
  double a[] = {
    x[0] * gsl_matrix_get(P[0], 1, 0) - gsl_matrix_get(P[0], 0, 0),
    x[0] * gsl_matrix_get(P[0], 1, 1) - gsl_matrix_get(P[0], 0, 1),
    x[0] * gsl_matrix_get(P[0], 1, 2) - gsl_matrix_get(P[0], 0, 2),
    x[1] * gsl_matrix_get(P[1], 1, 0) - gsl_matrix_get(P[1], 0, 0),
    x[1] * gsl_matrix_get(P[1], 1, 1) - gsl_matrix_get(P[1], 0, 1),
    x[1] * gsl_matrix_get(P[1], 1, 2) - gsl_matrix_get(P[1], 0, 2),
    0, 0, 0   /* work around unimplemented M < N */                 };
  double v[9], s[3], w[3];

  gsl_matrix_view A = gsl_matrix_view_array(a, 3, 3);
  gsl_matrix_view V = gsl_matrix_view_array(v, 3, 3);
  gsl_vector_view S = gsl_vector_view_array(s, 3);  
  gsl_vector_view W = gsl_vector_view_array(w, 3);  

  gsl_linalg_SV_decomp(&A.matrix, &V.matrix, &S.vector, &W.vector);
  // switched; retinas are on y axis:
  egoX[Y] = gsl_matrix_get(&V.matrix, 0, 2) / gsl_matrix_get(&V.matrix, 2, 2);
  egoX[X] = gsl_matrix_get(&V.matrix, 1, 2) / gsl_matrix_get(&V.matrix, 2, 2);
}


static int detectLeftGradient(const int image[NPIXELS]) {
  int i;
  for (i = NPIXELS - 1; i >= 0; i--)
    if (image[i] == OBJECT_COLOR)
      return i;
  return UNSEEN;
}


static int detectRightGradient(const int image[NPIXELS]) {
  int i;
  for (i = 0; i < NPIXELS; i++)
    if (image[i] == OBJECT_COLOR)
      return i;
  return UNSEEN;
}


void triangulate_egocentric(const Eye eyes[2], double Xego[2][2]) {
  int gradientPos[2][NEYES];
  gsl_matrix* P[NEYES];
  int eye;
  for (eye = 0; eye < NEYES; eye++) {
    P[eye] = getP(&eyes[eye]);
    gradientPos[0][eye] = detectLeftGradient(eyes[eye].image);
    gradientPos[1][eye] = detectRightGradient(eyes[eye].image);
  }
#if 0
  gradientPos[0][0] = 0;	// yields...
  gradientPos[0][1] = NPIXELS;  // ... cyclopean dist = BASELINE
  gradientPos[1][0] = NPIXELS * 3 / 7;
  gradientPos[1][1] = NPIXELS * 4 / 7;
#endif

  int feature;
  for (feature = 0; feature < 2; feature++) {
    if (gradientPos[feature][0] != UNSEEN &&
	gradientPos[feature][1] != UNSEEN   )
      getX(P, gradientPos[feature], Xego[feature]);
    else
      Xego[feature][0] = Xego[feature][1] = 0;
  }

  for (eye = 0; eye < NEYES; eye++)
    gsl_matrix_free(P[eye]);
}
