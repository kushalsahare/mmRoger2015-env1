/*************************************************************************/
/* File:        project9.c                                               */
/* Description: User project #9 - Velocity controlled harmoinc functions */
/* Author:      Mitchell Hebert                              		 */
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

#define PHI_MAX 0.20

extern double Kp_base_rot;
extern double Kp_base_trans;
extern double Kd_base_trans;

double omega_n;
//static double Mbase = (0.5+MARM_1+MARM_2);    /* kg  ( note: I = m*l^2) */
//static double Ibase = ((0.5+MARM_1+MARM_2)*SQR(R_BASE));      /* kg m^2 */

double* current_path_points;
int goal[2];
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

void updateControlStep(), sor() , mark_used() , x_draw_line() , project8_control(), project8_visualize();
double compute_gradient(), cell_distance();
double* get_current_goal();

void updateCurrentPath(Robot* roger){

  int i, j, xbin, ybin, already_used[NBINS][NBINS];
  double compute_gradient(), mag, grad[2], x, y;
  double distance = 0.0;
	
  path_size = 0;
  sor(roger);
	
  for (j=0; j<NBINS; ++j) {
    for (i=0; i<NBINS; ++i) {
      already_used[i][j] = FALSE;
    }
  }

  double x1 = roger->base_position[X];
  double y1 = roger->base_position[Y];
	
  i = (int)((MAX_Y - y1)/YDELTA);
  j = (int)((x1 - MIN_X)/XDELTA);
    
  ybin = i; xbin = j;
    
  // follow a stream line
  x = MIN_X + (j+0.05)*XDELTA;
  y = MAX_Y - (i+0.05)*YDELTA;
        

  if (!already_used[ybin][xbin]) {
    int loops = 0;
    while ((roger->world_map.occupancy_map[ybin][xbin] != GOAL) &&
	   (loops++ < 1000)) {
      mag = compute_gradient(x, y, roger, grad);
      if (mag < THRESHOLD) {
	//TODO: Prevent uninitialized harmonic map to try to print stream
	//printf("gradient magnitude is too small %6.4lf\n", mag);
	}
      else {
	x_draw_line(BLUE, x, y, 
		    x-PATH_STEP_SIZE*grad[0], y-PATH_STEP_SIZE*grad[1]);
	distance += sqrt( pow( (x-PATH_STEP_SIZE*grad[0] - x) , 2) 
			  + pow(y-PATH_STEP_SIZE*grad[1]-y , 2) );
	path_distances[path_size] = 
	  sqrt( pow( (x-PATH_STEP_SIZE*grad[0] - x) , 2) 
		+ pow(y-PATH_STEP_SIZE*grad[1]-y , 2));
	path_headings[path_size] = atan2(-grad[1] , -grad[0]);
				
	if(path_headings[path_size] > M_PI / 2.0){
	  path_headings[path_size] = path_headings[path_size] - 2.0* M_PI;
	}

	x -= PATH_STEP_SIZE*grad[0];
	y -= PATH_STEP_SIZE*grad[1];
                
	path_x_points[path_size] = x;
	path_y_points[path_size] = y;
  
	path_size++;
				
	ybin = (int)((MAX_Y-y)/YDELTA);
	xbin = (int)((x-MIN_X)/XDELTA);
				
	// At goal
	if (mag < 0.001) {
	  //           break;
	}
      }
    }
    goal[0] = xbin;
    goal[1] = ybin;
    mark_used((i+1), (j+1), already_used);
  }
}

void calculatePathDtheta(Robot* roger){
  int i;
  for(i = 1 ; i < path_size; i++){
    path_dtheta[i] = path_headings[i+1]  - path_headings[i-1];
    if(path_dtheta[i] < 0){
      path_dtheta[i] = path_dtheta[i] * -1.0;
    }
  }
}

void setVelocities(Robot* roger){
  int i;
  double magnitude = 0.0;
	
  for(i = 0;  i < path_size ; i++){
    magnitude = (path_dtheta[i]) * ( 1 / path_distances[i]) ;
    path_velocities_max[i] =  ( PHI_MAX * omega_n ) / magnitude; 	
    if(path_velocities_max[i] > 10){
      path_velocities_max[i] = 10;
    }
  }
}

double computeMaxA(roger , velocity , i)
Robot* roger;
double velocity;
int i;
{
  double returnVal = -1*(velocity * 10 - WHEEL_W0)*(WHEEL_TS/WHEEL_W0);
  return ((returnVal)) / M_BASE;
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
	
  //Print Path Data
  //for(k = 0 ; k <= path_size ; k++){
  //   printf("%i , PLAN %f MAX %f DTHETA %f DISTANCE %f \n", 
  //      k, path_velocity_plan[k], path_velocities_max[k], path_dtheta[k], 
  //      path_distances[k] );
  //}
}

double computeNewControlStep(Robot* roger){
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
  if(f != f){    // what does this do?
    f = 0.0;
  }
	
  double desiredVelocity = path_velocity_plan[1];
  return (f + Kd_base_trans * desiredVelocity) / Kp_base_trans ;	
}

void project9_control(roger, time)
Robot* roger;
double time;
{ 
  omega_n = sqrt(Kp_base_rot/I_BASE);  
  updateCurrentPath(roger);
  calculatePathDtheta(roger);
  setVelocities(roger);
  EVR(roger);

  updateControlStep(computeNewControlStep(roger));
  project8_control(roger,time);


  double x1 = roger->base_position[X];
  double y1 = roger->base_position[Y];

  if (cell_distance((int)((x1 - MIN_X)/XDELTA), (int)((MAX_Y - y1)/YDELTA),
		    goal[0], goal[1]) < 0.05) {
    roger->world_map.color_map[goal[1]][goal[0]] = NOFILL;
    roger->world_map.occupancy_map[goal[1]][goal[0]] = FREESPACE;
    printf("AT GOAL BIN x %i , y %i  \n" ,goal[0] , goal[1]);
  }	
}

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project9_enter_params() 
{
  printf("Project 9 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ 
  project8_visualize(roger);
}

double cell_distance(xbin1, ybin1, xbin2, ybin2)
int xbin1, ybin1, xbin2, ybin2;
{
  double dist[2] = {0.0, 0.0};
    
  //calculate distance in x direction
  dist[X] = xbin2 - xbin1;
    
  if (fabs(dist[X]) <= 1) {
    dist[X] = 0;
  }
  else{
    dist[X] -= 1*SGN(dist[X]);
    dist[X] = dist[X]*XDELTA;
  }
  //calculate distance in y direction
  dist[Y] = ybin2 - ybin1;
    
  if (fabs(dist[Y]) <= 1) {
    dist[Y] = 0;
  }
  else{
    dist[Y] -= 1*SGN(dist[Y]);
    dist[Y] = dist[Y]*YDELTA;
  }
  return sqrt( SQR(dist[X]) + SQR(dist[Y]) );
}

