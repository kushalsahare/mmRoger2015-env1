/**************************************************************************/
/* File:        wall.h                                                     */
/* Description:  wall structure for robot                                  */
/* Author:     Kushal Sahare                                             */
/* Date:        3-18-2017                                                 */
/**************************************************************************/
#ifndef WALL_H_
#define WALL_H_

#include "geometry.h"
#include "Roger.h"
#include "simulate.h"
#define STATIONARY 1

typedef struct _wall{
  int axis;         /* XAXIS, YAXIS, ZAXIS */
  double mass;         // intrinsic parameters 
  double centroid[2];  // position of the centroid of the object
  double vertices[4][2];
  double velocity[2];  // velocity of the centroid of the object
  double ext_force[2]; // written by the simulator: endpoint load
  double theta;
  double theta_dot;
  double iTj[3][3]; // Transformation wrt inertial frame
  } Wall;

#endif //WALL_H_
