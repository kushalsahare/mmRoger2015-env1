/*
 * =====================================================================================
 *
 *       Filename:  arm.h
 *    Description:  This file contains the arm structure 
 *        Created:  04/21/2017 12:01:14 PM
 *       Revision:  none
 *       Compiler:  gcc
 *         Author:  Kushal Sahare 
 *   Organization:  Laboratory of Perceptual Robotics 
 *
 * =====================================================================================
 */
#ifndef ARM_H_
#define ARM_H_

typedef struct _arm {
  double iTj[4][4];
  int dof_type;                // revolute or prismatic type
  int axis;                    // XAXIS, YAXIS, ZAXIS
  double theta;
  double theta_dot;
  double torque;
  double extForce[2];          // (fx,fy) force on distal endpoint of this link
} Arm;

#endif //ARM_H_
