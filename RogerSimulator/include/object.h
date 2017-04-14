/**************************************************************************/
/* File:       kushal Sahare                                              */ 
/* Description: object structure to simulate object                       */
/* Author:      Kushal Sahare                                             */
/* Date:        11-1-2009                                                 */
/**************************************************************************/

typedef struct _object{
  int v;                 // no of vertices  
  int e;                 // no of edges
  double mass;           // mass of the object
  double Iz;             // moment of inertia
  double iTj[4][4];      
  int axis;              // axis of rotation
  double position[3];    // position of the center of the object
  double velocity[3];
  double net_extForce[3];
} Object;

