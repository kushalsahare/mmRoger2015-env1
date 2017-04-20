/*************************************************************************/
/* File:        velocity_control.c                                       */
/* Description: Velocity controlled path traversal                       */
/* Author:      Mitchell Hebert                                          */
/* Date:        01-20-2015                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

#define VELOCITY_CONTROL      TRUE
#define DEFAULT_CONTROL_STEP   0.2

#define PHI_MAX 0.05

extern double Kp_base_rot;
extern double Kd_base_rot;
extern double Kp_base_trans;
extern double Kd_base_trans;

double omega_n;
//static double Mbase = (0.5+MARM_1+MARM_2);  /* kg  ( note: I = m*l^2) */    
//static double Ibase = ((0.5+MARM_1+MARM_2)*SQR(R_BASE));    /* kg m^2 */    

double* current_path_points;
int path_size;
double PATH_STEP_SIZE = 0.01;
int initialized = 0;

double path_headings[NBINS*NBINS];
double path_dtheta[NBINS*NBINS];
double path_distances[NBINS*NBINS];
double path_x_points[NBINS*NBINS];
double path_y_points[NBINS*NBINS];
double path_velocities_max[NBINS*NBINS];
double *path_velocity_plan;

void updateControlStep(), sor(), mark_used(), x_draw_line();
double compute_gradient();

double velocity_control(roger)
Robot *roger;
{
  double computeNewControlStep();

  if (VELOCITY_CONTROL) { 
    return(computeNewControlStep(roger));
    // return(4.0*default_step);
  }
  else{
    return(DEFAULT_CONTROL_STEP);
  }
}

void updateCurrentPath(roger)
Robot * roger;
{
  int i, j, xbin, ybin, already_used[NBINS][NBINS];
  double compute_gradient(), mag, grad[2], x, y;
  double distance = 0.0;

  path_size = 0;
  sor(roger);

  x = roger->base_position[X];
  y = roger->base_position[Y];
  ybin = (int)((MAX_Y - y)/YDELTA);
  xbin = (int)((x - MIN_X)/XDELTA);

  // follow a stream line
  int loops = 0;
  while ((roger->world_map.occupancy_map[ybin][xbin] != GOAL) &&
	 (loops++ < 1000)) {
    mag = compute_gradient(x, y, roger, grad);
    if (mag < THRESHOLD) {
    //    TODO: Prevent uninitialized harmonic map to try to print stream
    //    printf("gradient magnitude is too small %6.4lf\n", mag);
    }
    else {
      distance += PATH_STEP_SIZE;
      path_distances[path_size] = PATH_STEP_SIZE;
      path_headings[path_size] = atan2(-grad[1] , -grad[0]);

      x -= PATH_STEP_SIZE*grad[0];
      y -= PATH_STEP_SIZE*grad[1];

      path_x_points[path_size] = x;
      path_y_points[path_size] = y;

      path_size++;

      ybin = (int)((MAX_Y-y)/YDELTA);
      xbin = (int)((x-MIN_X)/XDELTA);
    }
  }
}

void calculatePathDtheta(Robot* roger){
  int i;
  for(i = 1; i < path_size; i++){
    path_dtheta[i] = path_headings[i]  - path_headings[i-1];
    if (path_dtheta[i] < 0){
      path_dtheta[i] = path_dtheta[i] * -1.0;
    }
  }
}

void setVelocities(Robot* roger){
  int i;
  double magnitude = 0.0;

  for(i = 0; i<path_size; i++){
    magnitude = path_dtheta[i] / path_distances[i] ;
    path_velocities_max[i] =  ( PHI_MAX * omega_n ) / magnitude;
    if(path_velocities_max[i] > 10.0){
      path_velocities_max[i] = 10.0;
    }
  }
}

double computeMaxA(roger , velocity , i)
Robot* roger;
double velocity;
int i;
{
  double returnVal = -1.0*(velocity * 10.0 - WHEEL_W0)*(WHEEL_TS/WHEEL_W0);
  return (returnVal/M_BASE);
}

void EVR(Robot* roger){
  int k;
  double max_a;
  path_velocity_plan = malloc(sizeof(path_velocities_max));
  memcpy(path_velocity_plan,&path_velocities_max,sizeof(path_velocities_max));
  path_velocity_plan[0] =
    roger->base_velocity[X]*cos(roger->base_position[THETA])
    + roger->base_velocity[Y]*sin(roger->base_position[THETA]);
  path_velocity_plan[path_size] = 0.0;

  for (k = 0; k < path_size; k++){
    max_a = computeMaxA(roger , path_velocity_plan[k] , k);
    double v = sqrt(SQR(path_velocity_plan[k]) + 2*max_a*path_distances[k]);
    if (v < path_velocity_plan[k+1]){
      path_velocity_plan[k+1] = v;
    }
  }
  for (k = path_size; k > 0; k--){
    max_a = computeMaxA(roger , path_velocity_plan[k] , k);
    double v = sqrt(SQR(path_velocity_plan[k]) + 2*max_a*path_distances[k]);
    if (path_velocity_plan[k-1] > v){
      path_velocity_plan[k-1] = v;
    }
  }
  // Print Path Data
  // for(k = 0 ; k <= path_size ; k++){
  //   printf("%i , PLAN %f MAX %f DTHETA %f DISTANCE %f \n",
  //      k, path_velocity_plan[k], path_velocities_max[k], path_dtheta[k],
  //      path_distances[k] );
  // }
}

double computeNewControlStep(Robot* roger){
  omega_n = sqrt(Kp_base_rot/I_BASE);
  updateCurrentPath(roger);
  calculatePathDtheta(roger);
  setVelocities(roger);
  EVR(roger);

  double rogerVelocity =
    roger->base_velocity[X]*cos(roger->base_position[THETA])
    + roger->base_velocity[Y]*sin(roger->base_position[THETA]);
  double rogerX = roger->base_position[X];
  double rogerY = roger->base_position[Y];
  int  i;

  double nextDis = sqrt( SQR(rogerX - path_x_points[1])
                         + SQR(rogerY - path_y_points[1]) );
  double a = ( SQR(path_velocity_plan[1]) -  SQR(rogerVelocity) )/(2*nextDis);

  double f = a * M_BASE;
  //  if(f != f){    // what does this do?
  //    f = 0.0;
  //  }

  double desiredVelocity = path_velocity_plan[1];
  return (f + Kd_base_trans * desiredVelocity) / Kp_base_trans ;
}

