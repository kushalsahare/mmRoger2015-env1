/**************************************************************************/
/* File:        object.h                                                  */ 
/* Description: object structure to simulate object                       */
/* Author:      Kushal Sahare                                             */
/* Date:        11-1-2009                                                 */
/**************************************************************************/
// a composite of N elastic spheres at the vertices of a regular polygon

#ifndef OBJECT_H_
#define OBJECT_H_

typedef struct _polyball {
  int id;                 // CIRCLE || TRIANGLE || RECT but any N can be used
  int N;                  // N vertices
  double **v;             // vertices Nx2
  double Rsphere;         // the radius of the elastic spheres
  double radius;        // the distance from the body frame to the center
                          // of the elastic sphere
  double mass;            // toal mass of the entire polyball
  double moi;             // moment of inertia ( total_mass * SQR(radius) )
  double position[3];     // position (x,y,theta) of the object
  double velocity[3];     // velocity of the object
  double net_extForce[3]; // from collisions with other objects
  double iTj[4][4];       // frame from local to world;
} PolyBall;

#endif //OBJECT_H_
