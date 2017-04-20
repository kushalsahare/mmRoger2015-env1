/**************************************************************************/
/* File:        geometry.h                                                */
/* Description: geometry primitives for robot                             */
/* Author:     Kushal Sahare                                              */
/* Date:        3-20-2017                                                 */
/**************************************************************************/


#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#define DIMENSION 2
#define MAX_SIZE 100
#define EPSILON 0.000001

typedef double point[DIMENSION]; // point object

typedef struct{
        int n;
        point p[MAX_SIZE]; // for square
} polygon;


typedef struct{
       double a;
       double b;
       double c;
} line ; // line defined by a, b and c

/* everything defined ckw*/

typedef point rect[4]; // rectangle
typedef point triangle[3] ; // triangle
//typedef point segment[2]; // segment

typedef struct {
    point c;        /* center of circle */
    double r;       /* radius of circle */
} circle;

typedef struct{
     point p1, p2;
} segment;        
#endif //GEOMETRY_H_
