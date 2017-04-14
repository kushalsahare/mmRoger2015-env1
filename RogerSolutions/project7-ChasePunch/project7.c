/*************************************************************************/
/* File:        project7.c - CHASE() PUNCH()                             */
/* Description: User project #7 - empty project directory for project    */
/*              development                                              */
/* Date:        03-30-2013                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int stereo_observation(), inv_arm_kinematics(), SEARCHTRACK();
void draw_observation();

Observation obs;

/* CHASE - the translational counterpart of SEARCHTRACK()               */
int CHASE(roger, time)
Robot* roger;
double time;
{
  double x_error, y_error;

  static int return_state = NO_REFERENCE;

  //  printf("inside CHASE:");
  //check if ball is in view
  if (stereo_observation(roger, &obs) == TRUE) {
    x_error = obs.pos[X] - roger->base_position[X];
    y_error = obs.pos[Y] - roger->base_position[Y];
    
    //    printf("error = %6.4lf %6.4lf  ", x_error, y_error);
    if (sqrt(SQR(x_error)+SQR(y_error)) < 0.4 ) return_state = CONVERGED;
    // leave setpoints unchanged

    else {
      return_state = TRANSIENT;
      roger->base_setpoint[X] = obs.pos[X];
      roger->base_setpoint[Y] = obs.pos[Y];
      //define_base_setpoint(roger, obs.pos[X], obs.pos[Y]);
    }
  }

  else {
    //set base translation goal
    roger->base_setpoint[X] = roger->base_position[X];
    roger->base_setpoint[Y] = roger->base_position[Y];

    return_state = NO_REFERENCE;
  }
  return return_state;
}

double arm_home[2][2] = {{(9.0*M_PI/10.0), (-9.0*M_PI/10.0)},
			 {(-9.0*M_PI/10.0), (9.0*M_PI/10.0)}};

/* PUNCH - striking the ball (any way possible)  **********************/
int PUNCH(roger, time)
Robot* roger;
double time;
{
  int i,j;
  static int return_state = NO_REFERENCE;

  //  printf("inside PUNCH:");

  if (stereo_observation(roger, &obs) == TRUE) {
    if (inv_arm_kinematics(roger, RIGHT, obs.pos[X],obs.pos[Y])) {
      return_state = TRANSIENT;
    }
    else {
      for (j=0; j<2; ++j) {
	roger->arm_setpoint[RIGHT][j] = arm_home[RIGHT][j];
      }
    }
    if (inv_arm_kinematics(roger, LEFT, obs.pos[X],obs.pos[Y])) {
      return_state = TRANSIENT;
    }
    else {
      for (j=0; j<2; ++j) {
	roger->arm_setpoint[LEFT][j] = arm_home[LEFT][j];
      }
    }
  }
  return(return_state);
}

/*********************************************************************/
int CHASEPUNCH(roger, time)
Robot* roger;
double time;
{
  int internal_state[3];
  static int return_state = NO_REFERENCE;

  /* policy for SEARCHTRACK, CHASE, PUNCH actions */

  internal_state[0] = SEARCHTRACK(roger,time);
  if ((internal_state[0]==TRANSIENT) || (internal_state[0] == CONVERGED)){
    internal_state[1] = CHASE(roger, time);
    internal_state[2] = PUNCH(roger, time);
  }

  return return_state;
}

/*********************************************************************/
int PRECISIONPUNCH(roger, time)
Robot* roger;
double time;
{
  int internal_state[3];
  static int return_state = NO_REFERENCE;

  /* policy combining SEARCHTRACK, APPROACH, KCONDITION (FOV, ISOARM)*/
  /* PRE-POSTURE, and STRAIGHT-PUNCH actions                         */

  internal_state[0] = SEARCHTRACK(roger,time);
  if ((internal_state[0]==TRANSIENT) || (internal_state[0] == CONVERGED)){
    internal_state[1] = CHASE(roger, time);
    internal_state[2] = PUNCH(roger, time);
  }

  return return_state;
}

/************************************************************************/

void project7_control(roger, time)
Robot* roger;
double time;
{
  int CHASEPUNCH();

  printf("CHASE status = %d\n", CHASEPUNCH(roger, time));
}

// user specified reset actions
void project7_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project7_enter_params() 
{
  printf("Project 4 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project7_visualize(roger)
Robot* roger;
{ 
  void draw_observation();
  draw_observation(obs); /* defined in xrobot.c */
}
