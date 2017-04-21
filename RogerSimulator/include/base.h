/*
 * =====================================================================================
 *
 *       Filename:  base.h
 *    Description:  This file contains the description of the base structure 
 *        Created:  04/21/2017 11:54:32 AM
 *       Revision:  none
 *       Compiler:  gcc
 *         Author:  Kushal Sahare (ksh.sah@gmail.com)
 *   Organization:  Laboratory of perceptual Robotics 
 *
 * =====================================================================================
 */
#ifndef BASE_H_
#define BASE_H_
typedef struct _base {
  double wTb[4][4];
  double x;
  double x_dot;
  double y;
  double y_dot;
  double theta;
  double theta_dot;
  double wheel_torque[2];
  double contact_theta;
  double extForce[2];          // net (fx,fy) force on the base
  double wheel_theta_dot[NWHEELS];
} Base;
#endif //BASE_H_
