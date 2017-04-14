/*************************************************************************/
/* File: project3.c                                                      */
/* Description: the signal processing code for Roger with oculomotor     */
/*              tracking (foveation) to provide a closed-loop evaluation */
/* Date:        9-1-2014                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

void rgb2hsi();

/*********  convert RGB to HSI  **********************************************/
void rgb2hsi(rgb, hsi)
int rgb[3][NPIXELS];
int hsi[3][NPIXELS];
{
  int i, nr, nl;
  double r,g,b,I,m,S,H;

  for (i=0;i<NPIXELS;++i) {
    /* PIXEL i IN LEFT EYE */
    r = (double) rgb[0][i];
    g = (double) rgb[1][i];
    b = (double) rgb[2][i];

    hsi[2][i] = (r + g + b)/3.0;
    m = MIN(MIN(r,g),b);
    hsi[1][i] = 1.0 - m/I;
    hsi[0][i] = acos((r-g/2.0-b/2.0)/sqrt(SQR(r)+SQR(g)+SQR(b)-r*g-r*b-g*b))
      * 180.0/M_PI;                                                  // degrees
    if (b>g) hsi[0][i] = 360.0-hsi[0][i];
  }
}    
    
void convolve_pair_1X3(f,g,h)
double f[NEYES][NPIXELS];
double g[3];
double h[NEYES][NPIXELS];
{
  int i, j, k, alpha;
  
  alpha = 1;
  h[LEFT][0] = h[LEFT][NPIXELS-1] = h[RIGHT][0] = h[RIGHT][NPIXELS-1] = 0;
  for (i=LEFT; i<=RIGHT; ++i) { 
    for (j=1; j<(NPIXELS-1); ++j) {
      h[i][j] = 0.0;
      for (k=-alpha; k<=alpha; ++k) {
	h[i][j] += g[k+alpha] * f[i][j+k];
      }
    }
  }
}

/****************************************************************************/
/* compute the average location (0<=pos<128) of red pixels on both images   */
/* if red is detected on both images:                                       */
/*             write the index of the mean pixel into ul and ur;            */
/*             return TRUE;                                                 */
/* else return FALSE;                                                       */
/****************************************************************************/
int compute_average_red_pixel(roger, ur, ul)
Robot* roger;
double *ur, *ul;
{
  int i, nr, nl;
  double r,g,b,I,m,S,H;

  nr = nl = 0;
  *ul = *ur = 0.0;

  for (i=0;i<NPIXELS;++i) {
    /* "red" means r=255 (it also means H=0) */
    if (roger->image[LEFT][i][RED_CHANNEL] == 255) { nl += 1; *ul += i;}
    if (roger->image[RIGHT][i][RED_CHANNEL] == 255) { nr += 1; *ur += i;}
  }

  if (nl>0) *ul /= (double) nl;
  if (nr>0) *ur /= (double) nr;
  if ((nl>0) && (nr>0)) {
    // printf(" ul=%6.4lf ur=%6.4lf\n", *ul, *ur);
    return(TRUE);
  }
  else return(FALSE);
}

double Prewit[3] = {-1, 0, 1};

/*************************************************************************/
// compute the (0<pos<128) of the average edge pixel on both image planes 
// if at least one edge is detected on both images: 
//    write the index of the mean pixel into ul and ur; return TRUE;
// else return FALSE;
/*************************************************************************/
int locate_edge_pixel(roger, ur, ul)
Robot * roger;
double *ur, *ul;
{
  int i;
  double r, g, b, I[NEYES][NPIXELS];
  double w, wsuml, wsumr, suml, sumr;
  double edge[NEYES][NPIXELS];

  // make the pair of intensity images
  for (i=0;i<NPIXELS;++i) {
    /* PIXEL i IN LEFT EYE */
    r = (double) roger->image[LEFT][i][RED_CHANNEL];
    g = (double) roger->image[LEFT][i][GREEN_CHANNEL];
    b = (double) roger->image[LEFT][i][BLUE_CHANNEL];

    I[LEFT][i] = (r + g + b)/3.0;

    /* PIXEL i IN RIGHT EYE */
    r = (double) roger->image[RIGHT][i][RED_CHANNEL];
    g = (double) roger->image[RIGHT][i][GREEN_CHANNEL];
    b = (double) roger->image[RIGHT][i][BLUE_CHANNEL];

    I[RIGHT][i] = (r + g + b)/3.0;
  }

  convolve_pair_1X3(I, Prewit, edge); 

  wsuml = wsumr = 0.0;
  suml = sumr = 0.0;
  *ul = *ur = 0.0;

  for (i=0;i<NPIXELS;++i) {
    w = fabs(edge[LEFT][i]);
    wsuml += w*i;
    suml += w;
    w = fabs(edge[RIGHT][i]);
    wsumr += w*i;
    sumr += w;
  }
  if (suml > 0) *ul = wsuml/suml;
  if (sumr > 0) *ur = wsumr/sumr;
  if ((suml>0) && (sumr>0)) return(TRUE);
  else return(FALSE);
}

#define RED_TRACK   0
#define EDGE_TRACK  1

/* executed automatically when                                           */
/* control mode = PROJECT3                                               */
void project3_control(roger, time)
Robot * roger;
double time;
{
  double ul,ur;
  
  if (((RED_TRACK) && (compute_average_red_pixel(roger, &ur, &ul))) ||
      ((EDGE_TRACK)&& (locate_edge_pixel(roger, &ur, &ul)))) {
      roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT] + 
	atan2((ul-63.5), FOCAL_LENGTH);
      if (roger->eyes_setpoint[LEFT]>(M_PI/2.0))
	roger->eyes_setpoint[LEFT]=(M_PI/2.0);
      if (roger->eyes_setpoint[LEFT]<(-M_PI/2.0))
	roger->eyes_setpoint[LEFT]=(-M_PI/2.0);

      roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] +
	atan2((ur-63.5), FOCAL_LENGTH);
      if (roger->eyes_setpoint[RIGHT]>(M_PI/2.0))
	roger->eyes_setpoint[RIGHT] = (M_PI/2.0);
      if (roger->eyes_setpoint[RIGHT]<(-M_PI/2.0))
	roger->eyes_setpoint[RIGHT] = (-M_PI/2.0);
  }
}


void project3_reset(roger)
Robot* roger;
{ }

void project3_enter_params() 
{
  printf("Project 2 enter_params called. \n");
}

void project3_visualize(roger)
Robot* roger;
{ }


