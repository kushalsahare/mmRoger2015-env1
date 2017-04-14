/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Kinematic function for Arms                              */
/*                 fwd_arm_kinematics() - maps joint angles in the arm   */
/*                    into endpoint coordinates in the base frame        */
/*                 inv_arm_kinematics() - maps endpoint position for the */
/*                    specified arm in base frame into two possible joint*/
/*                    angle configurations and selects one of the two    */
/*                    writes arm cspace setpoints                        */
/* Date:        1-2015                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

/*************************************************************************/
/*** PROJECT #2 - FORWARD KINEMATICS: MAP (THETA1, THETA2) TO (X,Y)_B  ***/
/***              INVERSE KINEMATICS: MAP (X,Y)_B TO (THETA1, THETA2)  ***/
/*************************************************************************/
void fwd_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{ }

int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  // input (x,y) is in world frame coordinates - map it into the base frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw);

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  // printf("x = %6.4lf  y = %6.4lf OUT OF REACH\n", ref_b[X], ref_b[Y]);
  return FALSE;
}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
/* executed automatically when                                           */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
void project2_control(roger, time)
Robot * roger;
double time;
{ }


void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
  printf("Project 6 enter_params called. \n");
}

void project2_visualize(roger)
Robot* roger;
{ }


