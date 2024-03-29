/****************************************************************/
/** kalman.c: kalman filter for tracking the ball              **/
/** author:   Grupen                                           **/
/** date:     Spring 2013                                      **/
/****************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

// initial noise/covariance scales defined in include/control.h
//#define SIGMA_OBS       0.01  // the observation covariance
//#define SIGMA_PROCESS 100.0   // the forward/process model   
//#define OBS_DT          0.001 // same as servo and render rate

//void matXvec4441(),matrix_transpose44(),matrix_transpose24(),matXmat4444();
//void matXmat4442(),matXmat2442(),matXmat4222(),matXmat4224(),matXvec2441();
//void matXvec4221(),matrix_sum22(),matrix_sum44(),vector_sum41();
//void invert_matrix22(), matrix_diff(), copy_matrix44();

int stereo_observation();

//double time = 0.0;

extern Observation obs;
Estimate x_plus, x_minus; // THE STATE

double IDENTITY[4][4] = {{1.0, 0.0, 0.0, 0.0},
			 {0.0, 1.0, 0.0, 0.0},
			 {0.0, 0.0, 1.0, 0.0},
			 {0.0, 0.0, 0.0, 1.0}};

// THE FORWARD MODEL x(k+1) = Ax(k) + Bu(k) + w     w~N(0,Qk)
// the default process uncertainty is SQR(SIGMA_PROCESS)*Qk
double A[4][4] = {{1.0, 0.0, OBS_DT, 0.0},
		  {0.0, 1.0, 0.0, OBS_DT},
		  {0.0, 0.0, 1.0, 0.0},
		  {0.0, 0.0, 0.0, 1.0}};
double Qk[4][4];

// SENSOR MODEL AND OBSERVATIONS z(k) = H x(k) + v     v~N(0,Rk)
// observe position z=[x, y]^T component of state, x, the default
// sensor uncertainty is SQR(SIGMA_OBS) * Rk
double H[2][4] = { {1, 0, 0, 0}, {0, 1, 0, 0} };
double Rk[2][2] = { { 1.0, 0.0 }, { 0.0, 1.0 } };

int KFinit_flag = TRUE;
double sigma_obs=SIGMA_OBS;         // in control.h
double sigma_process=SIGMA_PROCESS;

/*****************************************************************************/
/*****                        Visualization                              *****/
/*****************************************************************************/
void plot_data()
{
  // ground truth
  //     printf("%lf %lf\n", time, reality[X]);
  //     printf("%lf %lf\n", time, reality[Y]);
  //     printf("%lf %lf\n", time, reality[XDOT]);
  //     printf("%lf %lf\n", time, reality[YDOT]);
  //if (KFinit_flag == TRUE) {
  //printf("===================================\n");
  //}
  // observations
  //printf("Observations:\n\tTime: %lf px: %lf py: %lf\n", 
  //        obs.time, obs.pos[X], obs.pos[Y]);
  //printf("\tCov:\n\t\t%lf %lf\n\t\t%lf %lf\n",
  //        obs.cov[0][0], obs.cov[0][1], obs.cov[1][0], obs.cov[1][1]);

  // estimate
  //printf("Estimate:\n\tTime: %lf px: %lf py: %lf\n",
  //        x_plus.time, x_plus.state[X], x_plus.state[Y]);
  //printf("\tCov:\n\t\t%lf %lf\n\t\t%lf %lf\n\n", 
  //     x_plus.cov[0][0],x_plus.cov[0][1],x_plus.cov[1][0],x_plus.cov[1][1]);
}

/*****************************************************************************/
/*****                     Gaussian noise generator                      *****/
/*****************************************************************************/
#ifndef MAX_RAND
#define MAX_RAND   2147483647.0
#endif

double xx[31] = {-3.0, -2.8, -2.6, -2.4, -2.2, -2.0, -1.8, -1.6, -1.4, -1.2,
		 -1.0, -0.8, -0.6, -0.4, -0.2,  0.0,  0.2,  0.4,  0.6,  0.8,
		 1.0,  1.2,  1.4,  1.6,  1.8,  2.0,  2.2,  2.4,  2.6,  2.8,
		 3.0};

double px[31] = {0.0013940399, 0.0026287699, 0.0047786120, 0.0083763619,
		 0.0141632743, 0.0231098302, 0.0364039426, 0.0553910419,
		 0.0814558719, 0.1158472240, 0.1594629019, 0.2126291295,
		 0.2749207450, 0.3450702193, 0.4210015026, 0.5000000000,
		 0.5789984974, 0.6549297807, 0.7250792550, 0.7873708705,
		 0.8405370981, 0.8841527760, 0.9185441281, 0.9446089581,
		 0.9635960574, 0.9768901698, 0.9858367257, 0.9916236381,
		 0.9952213880, 0.9973712301, 0.9986059601};

/*** Returns random variable with normal dist., mean = 0 and variance=sigma***/
double gauss_noise(sigma)
double sigma;
{
  double x,noise;
  int    i;

  x = ((double) random ())/MAX_RAND;
  for (i=0; i<31; i++) if (x < px[i]) break;
  if (i == 30) noise = 3.0*sigma;
  else if (i == 0) noise = -3.0*sigma;
  else noise = (xx[i-1]+(x-px[i-1])*(xx[i]-xx[i-1])/(px[i]-px[i-1]))*sigma;
  return(noise);
}

// called from parent process once before running filter
void init_filter_parameters()
{
  double cov2, cov3, cov4;

  cov4 = (pow(sigma_process,2.0)*pow(OBS_DT,4.0)/4.0);
  cov3 = (pow(sigma_process,2.0)*pow(OBS_DT,3)/2.0);
  cov2 = (pow(sigma_process,2.0)*pow(OBS_DT,2));
  Qk[0][0]=cov4; Qk[0][1]=0.00; Qk[0][2]=cov3; Qk[0][3]=0.00;
  Qk[1][0]=0.00; Qk[1][1]=cov4; Qk[1][2]=0.00; Qk[1][3]=cov3;
  Qk[2][0]=cov3; Qk[2][1]=0.00; Qk[2][2]=cov2; Qk[2][3]=0.00;
  Qk[3][0]=0.00; Qk[3][1]=cov3; Qk[3][2]=0.00; Qk[3][3]=cov2;
}

// this seems to work ok
void initialize_state(obs, time)
Observation obs;
double time;
{
  double cov2, cov3, cov4;

  x_plus.state[X]=obs.pos[X]; x_plus.state[Y]=obs.pos[Y];
  x_plus.state[XDOT]=0.0; x_plus.state[YDOT]=0.0; // generally non-zero

  //  copy_matrix44(Qk, x_plus.cov);
  matrix_copy(4, 4, Qk, x_plus.cov);
  x_plus.time = time;
}

// initialize the process using the obs covariance
// works well *** more correct ***
void initialize_state2(obs, time)
Observation obs;
double time;
{
  x_plus.state[X]=obs.pos[X]; x_plus.state[Y]=obs.pos[Y];
  x_plus.state[XDOT]=0.0; x_plus.state[YDOT]=0.0; // generally non-zero
  
  x_plus.cov[0][0] = obs.cov[0][0]; x_plus.cov[0][1] = obs.cov[0][1];
  x_plus.cov[0][2] = x_plus.cov[0][3] = 0.0;

  x_plus.cov[1][0] = obs.cov[1][0]; x_plus.cov[1][1] = obs.cov[1][1];
  x_plus.cov[1][2] = x_plus.cov[1][3] = 0.0;

  x_plus.cov[2][0] = 0.0; x_plus.cov[2][1] = 0.0;
  x_plus.cov[2][2] = 1.0; x_plus.cov[2][3] = 0.0;

  x_plus.cov[3][0] = 0.0; x_plus.cov[3][1] = 0.0;
  x_plus.cov[3][2] = 0.0; x_plus.cov[3][3] = 1.0;

  x_plus.time = time;
}

void kalman_filter(roger, time)
Robot * roger;
double time;
{
  int i,j;
  double R[2][2], AT[4][4], HT[4][2], Kk[4][2];
  double tmp_mat1_44[4][4], tmp_mat2_44[4][4];
  double tmp_mat1_22[2][2], tmp_mat2_22[2][2], tmp_mat1_42[4][2];
  double tmp_vec1_21[2], tmp_vec2_21[2], tmp_vec1_41[4][1];
  double gauss_noise();
  static int count;

  // EXTRAPOLATE: x' = Ax + Bu + w    w~N(0,Qk)
  // no B terms are implemented
  matrix_mult(4,4,A, 1,x_plus.state, x_minus.state);
  matrix_transpose(4,4,A,AT);
  matrix_mult(4,4,x_plus.cov, 4, AT, tmp_mat1_44);
  matrix_mult(4,4,A, 4, tmp_mat1_44, tmp_mat2_44);
  matrix_add(4,4,tmp_mat2_44, Qk, x_minus.cov);

  // stereo_observation() returns TRUE when there is a "valid" Cartesian
  // observation by the criteria in project2/vision/stereo_observation()
  if (stereo_observation(roger, &obs)==FALSE) { // no valid stereo observation
    for (i=0;i<4; ++i) {
      x_plus.state[i] = x_minus.state[i];
      for (j=0;j<4; ++j) {
	x_plus.cov[i][j] = x_minus.cov[i][j];
      }
    }
  }
  else {
    if (KFinit_flag == TRUE) {
      initialize_state(obs, time); // based on disturbance acc.
      //initialize_state2(obs, time);        // based on observation
      KFinit_flag = FALSE;
    }
    else {
      // UPDATE : Kalman gain
      matrix_transpose(2,4,H, HT);
      matrix_mult(4,4,x_minus.cov, 2,HT, tmp_mat1_42);
      matrix_mult(2,4,H, 2,tmp_mat1_42, tmp_mat1_22);
      
      // default case:
      R[0][0] = SQR(sigma_obs)*Rk[0][0];
      R[0][1] = SQR(sigma_obs)*Rk[0][1];
      R[1][0] = SQR(sigma_obs)*Rk[1][0];
      R[1][1] = SQR(sigma_obs)*Rk[1][1];
      matrix_add(2,2,tmp_mat1_22, R, tmp_mat2_22);
      
      //uses localizability ellipsoid
      //matrix_add(2,2, tmp_mat1_22, obs.cov, tmp_mat2_22);
      matrix_invert(tmp_mat2_22, 2, tmp_mat1_22);

      matrix_mult(4,2,HT, 2,tmp_mat1_22, tmp_mat1_42);
      matrix_mult(4,4,x_minus.cov, 2,tmp_mat1_42, Kk);

      //  printf("    Kk = %6.4lf %6.4lf \n",  Kk[0][0],Kk[0][1]);
      //  printf("         %6.4lf %6.4lf \n",  Kk[1][0],Kk[1][1]);
      //  printf("         %6.4lf %6.4lf \n",  Kk[2][0],Kk[2][1]);
      //  printf("         %6.4lf %6.4lf \n\n",  Kk[3][0],Kk[3][1]);

      matrix_mult(2,4,H, 1,x_minus.state, tmp_vec1_21);// predicted observation

      // update estimate based on zk
      tmp_vec2_21[0] = obs.pos[0] - tmp_vec1_21[0];    // real observation
      tmp_vec2_21[1] = obs.pos[1] - tmp_vec1_21[1];

      matrix_mult(4,2,Kk, 1,tmp_vec2_21, tmp_vec1_41);
      matrix_add(4,1,x_minus.state, tmp_vec1_41, x_plus.state);
      matrix_mult(4,2,Kk, 4,H, tmp_mat1_44);
      matrix_subtract(4,4,IDENTITY, tmp_mat1_44, tmp_mat1_44);
      matrix_mult(4,4,tmp_mat1_44, 4,x_minus.cov, x_plus.cov);
      x_plus.time = time;
    }
  }
  plot_data();
}
