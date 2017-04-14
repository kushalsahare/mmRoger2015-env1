/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - Localize development environment       */
/* Date:        12-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

int stereo_observation();
void draw_observation();

extern Observation obs;

// triangulate the position of the red ball, transform from the base frame to
// to world frame and write into Observation obs
int stereo_observation(roger, obs)
Robot * roger;
Observation * obs;
{
  double ur, ul;
  double wTb[4][4], bTw[4][4], wRb[2][2], bRw[2][2], ref_b[4], ref_w[4];
  double lambdaL, gammaL, gammaR; // stereoJJt[2][2];
  double mat_22[2][2], cov_b[2][2];

  int compute_average_red_pixel();
  static int no_red_blue_transitions = TRUE;
  int check_stereo_FOV();
  void stereoJJT();
  
  /************************************************************************/
  // PROJECT2: triangulate to localize the red ball in the base frame
  //           convert to world frame, and write into Observation "obs"
  //
  // use filters to eliminate "invalid" stereo observations: is there "red"
  // on both image planes? are the hands occluding the ball (red-blue
  // transitions? is the ball leaving the image plane (any FOV issues)?
  if ((compute_average_red_pixel(roger, &ur, &ul) == TRUE) &&
      //      (no_red_blue_transitions) && (TRUE) ) {
      (no_red_blue_transitions) && (check_stereo_FOV(roger) == TRUE) ) {

    // get angle from base to object
    gammaL = roger->eye_theta[LEFT] + atan2((ul - 63.5), 64.0);
    gammaR = roger->eye_theta[RIGHT] + atan2((ur - 63.5), 64.0);

    // lambda_L: catch case where eyes are parallel and same pixels on both
    // eyes (infinite distance)
    if ((gammaR - gammaL) == 0.0) lambdaL = 20.0;
    else lambdaL = 2.0*BASELINE*cos(gammaR) / sin(gammaR - gammaL);

    // calculate x,y coordinate in base frame
    ref_b[X] = lambdaL * cos(gammaL);
    ref_b[Y] = BASELINE + lambdaL * sin(gammaL);
    ref_b[2] = 0.0;
    ref_b[3] = 1.0;
    // convert into world frame
    construct_wTb(roger->base_position, wTb);
    wRb[0][0] = wTb[0][0]; wRb[0][1] = wTb[0][1];
    wRb[1][0] = wTb[1][0]; wRb[1][1] = wTb[1][1];

    matrix_mult(4, 4, wTb, 1, ref_b, ref_w);

    obs->pos[X] = ref_w[X];
    obs->pos[Y] = ref_w[Y];

    // compute observation cov (JJT) 
    stereoJJT(roger, ur, ul, cov_b);
    // and rotate it into world coordinates ... [cov]_w = wRb [cov]_b wRb^T
    //    matrix_transpose22(wRb, bRw);
    matrix_transpose(2, 2, wRb, bRw);
    //    matXmat2222(wRb, cov_b, mat_22);
    matrix_mult(2, 2, wRb, 2, cov_b, mat_22);
    //    matXmat2222(mat_22, bRw, obs->cov);
    matrix_mult(2, 2, mat_22, 2, bRw, obs->cov);

    //obs->cov[0][0] = cov_b[0][0];
    //obs->cov[0][1] = cov_b[0][1];
    //obs->cov[1][0] = cov_b[1][0];
    //obs->cov[1][1] = cov_b[1][1];

    obs->cov[0][0] *= SQR(SIGMA_OBS);
    obs->cov[0][1] *= SQR(SIGMA_OBS);
    obs->cov[1][0] *= SQR(SIGMA_OBS);
    obs->cov[1][1] *= SQR(SIGMA_OBS);

    //  printf("inside stereo_observation()\n");
    //  printf("x=%6.4lf y=%6.4lf\n", obs->pos[X], obs->pos[Y]);
    //  printf("   %lf %lf\n", obs->cov[0][0], obs->cov[0][1]);
    //  printf("   %lf %lf\n", obs->cov[1][0], obs->cov[1][1]);

    return(TRUE);
  }
  else return(FALSE);
}

/*************************************************************************/
// if red is detected in pixel 0 and 127 on either image:
//       return FALSE
// else  return TRUE;
/*************************************************************************/
int check_stereo_FOV(roger)
Robot* roger;
{
  int i, p[2][2]={{FALSE, FALSE},{FALSE, FALSE}};

  double r, g, b, I, m, S, H;

  /* PIXEL 0 IN LEFT EYE */
  r = (double) roger->image[LEFT][0][RED_CHANNEL];
  g = (double) roger->image[LEFT][0][GREEN_CHANNEL];
  b = (double) roger->image[LEFT][0][BLUE_CHANNEL];

  //  I = (r + g +  b)/3.0;
  //  m = MIN(MIN(r,g),b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;
  
  // this is "red"
  if ( (H<=10.0) || H>=350.0) { p[LEFT][0]=TRUE; }

  /* PIXEL 127 IN LEFT EYE */
  r = (double) roger->image[LEFT][127][RED_CHANNEL];
  g = (double) roger->image[LEFT][127][GREEN_CHANNEL];
  b = (double) roger->image[LEFT][127][BLUE_CHANNEL];

  //  I = (r + g +  b)/3.0;
  //  m = MIN(MIN(r,g),b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;

  // this is "red"
  if ( (H<=2.0) || H>=358.0) { p[LEFT][1]=TRUE; }

  /* PIXEL 0 IN RIGHT EYE */
  r = (double) roger->image[RIGHT][0][RED_CHANNEL];
  g = (double) roger->image[RIGHT][0][GREEN_CHANNEL];
  b = (double) roger->image[RIGHT][0][BLUE_CHANNEL];

  //  I = (r + g + b)/3.0;
  //  m = MIN(MIN(r,g), b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;

  if ( (H<=2.0) || H>=358.0) { p[RIGHT][0]=TRUE; }

  /* PIXEL 127 IN RIGHT EYE */
  r = (double) roger->image[RIGHT][127][RED_CHANNEL];
  g = (double) roger->image[RIGHT][127][GREEN_CHANNEL];
  b = (double) roger->image[RIGHT][127][BLUE_CHANNEL];

  //  I = (r + g + b)/3.0;
  //  m = MIN(MIN(r,g), b);
  //  S = 1.0 - m/I;
  H = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b)) // radians
    * 180.0/M_PI;                                                // degrees
  if (b>g) H=360.0-H;

  if ( (H<=2.0) || H>=358.0) { p[RIGHT][1]=TRUE; }

  /***********************************************************************/

  if (p[LEFT][0] || p[LEFT][1] || p[RIGHT][0] || p[RIGHT][1]) return(FALSE);
  else return(TRUE);
}

void stereoJJT(roger, ur, ul, JJT)
Robot * roger;
double ur, ul, JJT[2][2]; /* observation covariance only assigns 2x2 */
{
  double coeff, J00, J01, J10, J11;
  double gl, gr, lambdaL;

  gl = roger->eye_theta[LEFT] + atan2((ul - 63.5), 64.0);
  gr = roger->eye_theta[RIGHT] + atan2((ur - 63.5), 64.0);

  if ((gr - gl) == 0.0) lambdaL = 20.0;
  else lambdaL = 2.0*BASELINE*cos(gr) / sin(gr - gl);

  // catch case where eyes are parallel and same pixels on both eyes (infinite
  // distance)                             
  if ((gr - gl) == 0.0) coeff = 20.0;
  else coeff = 2.0*BASELINE / SQR(sin(gr-gl));

  // calculate x,y coordinate in base frame
  //J00 = coeff * sin(gr)*cos(gr);
  //J01 = -coeff * sin(gl)*cos(gl);
  //J10 = coeff * SQR(sin(gr));
  //J11 = -coeff * SQR(sin(gl));

  J00 = coeff * cos(gr)*cos(gr);
  J01 = -coeff * cos(gl)*cos(gl);
  J10 = coeff * sin(gr)*cos(gr);
  J11 = -coeff * sin(gl)*cos(gl);

  //  printf(" ** J00=%6.4lf J01=%6.4lf J10=%6.4lf J11=%6.4lf\n",
  //         J00,J01,J10,J11);
  JJT[0][0] = SQR(J00) + SQR(J01);
  JJT[0][1] = JJT[1][0] = J00*J10 + J01*J11;
  JJT[1][1] = SQR(J10) + SQR(J11);
  //  printf(" ** JJT00=%6.4lf JJT01=%6.4lf JJT10=%6.4lf JJT11=%6.4lf\n",
  //     JJT[0][0], JJT[0][1], JJT[1][0], JJT[1][1]);       
}

/*****************************************************************************/
void project5_control(roger, time)
Robot* roger;
double time;
{
  int stereo_observation(), SEARCHTRACK();
  int ST_return;

  ST_return = SEARCHTRACK(roger, time);
  if ((ST_return==TRANSIENT) || (ST_return==CONVERGED)) {
    printf("stereo_observation() state=%d\n", stereo_observation(roger, &obs));
  }
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
  printf("Project 5 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{ 
  void draw_observation();
  draw_observation(obs); /* defined in xrobot.c */
}
