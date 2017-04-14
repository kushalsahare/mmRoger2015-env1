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

#define VELOCITY_CONTROL    TRUE

#define PHI_MAX 0.20 // the limiting (omega/omega_n)

extern double Kp_base_rot;
extern double Kd_base_rot;
extern double Kp_base_trans;
extern double Kd_base_trans;

double *path_velocity_plan;
extern double omega_n;
#define BIGPATH   1000

double ds[BIGPATH]; // reserve a big enough memory for worst case
double path[3][BIGPATH]; // X, Y, and THETA

int npts;
double PATH_STEP_SIZE = 0.01;
double path_velocities_max[NBINS*NBINS];

double velocity_control(default_step, roger)
double default_step;
Robot *roger;
{
  double computeNewControlStep();

  if (VELOCITY_CONTROL) { 
    //
    //    return(4.0*default_step);
    //
    return(computeNewControlStep(roger));
  }
  else{
    return(default_step);
  }
}

double computeNewControlStep(Robot* roger){
  double rogerVelocity =
    roger->base_velocity[X]*cos(roger->base_position[THETA])
    + roger->base_velocity[Y]*sin(roger->base_position[THETA]);
  double rogerX = roger->base_position[X];
  double rogerY = roger->base_position[Y];
  int  i;

  double nextDis = sqrt( SQR(rogerX - path[X][1])
			 + SQR(rogerY - path[Y][1]) );
  double a=(SQR(path_velocity_plan[1])-SQR(rogerVelocity))/(2.0*nextDis);

  double f = a * M_BASE;
  if(f != f){    // what does this do?
    f = 0.0;
  }
      
  double desiredVelocity = path_velocity_plan[1];
  return ((f + Kd_base_trans * desiredVelocity) / Kp_base_trans);
}

/****************************************************************************/
/****************************************************************************/

void generateCurrentStreamline(roger)
Robot * roger;
{
  //  int i, j, xbin, ybin, already_used[NBINS][NBINS];
  void sor(), x_draw_line();

  int xbin, ybin, path_index, npts;
  double compute_gradient(), mag, grad[2], x, y;
  double cum_distance = 0.0;

  sor(roger);

  path[X][0]=roger->base_position[X]; xbin=(int)((path[X][0]-MIN_X)/XDELTA);
  path[Y][0]=roger->base_position[Y]; ybin=(int)((MAX_Y-path[Y][0])/YDELTA);
  ds[0] = cum_distance = 0.0;

  npts = path_index = 1; // points to the end of the via point list
  while ((roger->world_map.occupancy_map[ybin][xbin] != GOAL) 
	 && (path_index++ < 1000)) {
    mag = compute_gradient(x, y, roger, grad);
    if (mag > THRESHOLD) {
      x_draw_line(BLUE, x, y,
		  (x-PATH_STEP_SIZE*grad[0]), (y-PATH_STEP_SIZE*grad[1]));

      ds[path_index] = PATH_STEP_SIZE;
      cum_distance += ds[path_index];
      path[X][path_index] = path[X][path_index-1] - PATH_STEP_SIZE*grad[X];
      path[Y][path_index] = path[Y][path_index-1] - PATH_STEP_SIZE*grad[Y];
      path[THETA][path_index] = atan2(-grad[Y] , -grad[X]);


      ybin = (int)((MAX_Y - path[Y][path_index]) / YDELTA);
      xbin = (int)((path[X][path_index] - MIN_X) / XDELTA);
    }
    path_index++;
  }
  npts = path_index;
}

#define DTHETA_DS_MIN 0.00001

void maxVelocities(roger) // based on curvature
Robot * roger;
{
  int i;
  double dtheta_ds;

  for (i=1; i<npts; i++) {
    dtheta_ds = fabs(path[THETA][i] - path[THETA][i-1])/ PATH_STEP_SIZE;
    dtheta_ds = MAX(dtheta_ds, DTHETA_DS_MIN); 
    path_velocities_max[i] =  ( PHI_MAX * omega_n ) / dtheta_ds;

    // determined from no load speed (omega_0*R_wheel)
    path_velocities_max[i] = MIN(path_velocities_max[i], (WHEEL_W0*R_WHEEL));
  }
}


// the current x_dot, y_dot, theta_dot -> wheel speeds -> motor models
// -> torque limits on each wheel -> maximum translational acceleration
double computeMaxA(roger , velocity , i)
Robot * roger;
double velocity;
int i;
{
  // Mitchell's code
  double returnVal = -1.0*(velocity * 10.0 - WHEEL_W0)*(WHEEL_TS/WHEEL_W0);
  return (returnVal/M_BASE);

  // from the RogerSimulator code in platform.c, procedure wheel_speed()
  //  wheel_theta_dot[LEFT] =
  //    (mobile_base->x_dot*cos(mobile_base->theta) +
  //     mobile_base->y_dot*sin(mobile_base->theta) -
  //     mobile_base->theta_dot * R_AXLE) / R_WHEEL;
  //
  //  wheel_theta_dot[RIGHT] =
  //    (mobile_base->x_dot*cos(mobile_base->theta) +
  //     mobile_base->y_dot*sin(mobile_base->theta) +
  //     mobile_base->theta_dot * R_AXLE) / R_WHEEL;

  // theta_ddot_des = Kp_base_rot*(theta_ref - theta_act) 
  //                     + Kd_base_rot*(theta_dot_ref - theta_dot_act);
  // Mdes = I_BASE*theta_ddot_des;
  // torque_rot[LEFT] = 
  // torque_rot[RIGHT] = 
  //
  // torque_rot_left = 
  //   motor_model(torque_tot[LEFT],wheel_theta_dot[LEFT],WHEEL_TS,WHEEL_W0);

  //  mobile_base.wheel_torque[0] =
  //  motor_model(Roger.wheel_torque[0], Roger.wheel_theta_dot[0],
  //	      WHEEL_TS, WHEEL_W0);
  //  mobile_base.wheel_torque[1] =
  //    motor_model(Roger.wheel_torque[1], Roger.wheel_theta_dot[1],
  //                WHEEL_TS, WHEEL_W0);

  // now turn it back into net rotational moment and net translational force
  //    -> net rotational and translational acceleration
  //       return translational acceleration limit
}

// Algorithm 2 in IROS paper
void VelocityReachability(Robot* roger)
{
  int k;
  double max_a;

  path_velocity_plan = malloc(sizeof(path_velocities_max));
  memcpy(path_velocity_plan,&path_velocities_max,sizeof(path_velocities_max));
  path_velocity_plan[0] =
    roger->base_velocity[X]*cos(roger->base_position[THETA])
    + roger->base_velocity[Y]*sin(roger->base_position[THETA]);
  path_velocity_plan[npts] = 0.0;

  for (k = 0; k < npts; k++){
    max_a = computeMaxA(roger , path_velocity_plan[k] , k);
    double v = sqrt(SQR(path_velocity_plan[k]) + 2*max_a*ds[k]);
    if (v < path_velocity_plan[k+1]){
      path_velocity_plan[k+1] = v;
    }
  }
  for (k = npts; k > 0; k--){
    max_a = computeMaxA(roger , path_velocity_plan[k] , k);
    double v = sqrt(SQR(path_velocity_plan[k]) + 2*max_a*ds[k]);
    if (path_velocity_plan[k-1] > v){
      path_velocity_plan[k-1] = v;
    }
  }

  // Print Path Data
  // for(k = 0 ; k <= npts ; k++){
  //   printf("%i , PLAN %f MAX %f DTHETA %f DISTANCE %f \n",
  //          k,path_velocity_plan[k],path_velocities_max[k],path_dtheta[k],
  //          ds[k] );
  // }
}

