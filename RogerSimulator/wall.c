/*************************************************************************/
/* File:        wall.c                                                 */
/* Description: all structures and dynamics specific to the wall      */
/* Author:      Kushal Sahare                                               */
/* Date:        3-19-2017                                                */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "wall.h"
#include "Roger.h"
#include "geometry.h"


extern double clock;
Wall wall= {ZAXIS, /*axis*/
	M_WALL, /*Mass*/ 
	{2.0,2.0}, /*centroid*/
	{ {2.0 - W_WALL*0.5, 2.0 + H_WALL*0.5 },
		{2.0 + W_WALL*0.5, 2.0 + H_WALL*0.5},
		{2.0 + W_WALL*0.5, 2.0 - H_WALL*0.5},
		{2.0 - W_WALL*0.5, 2.0 - H_WALL*0.5}}, /*vertices*/
	{0.0,0.0}, /*velocity*/
	{0.0,0.0}, /*ext_force*/
	0.0, /*theta*/
	0.0, /*theta_dot*/
	{{1.0,0.0, 2.0},
		{0.0,1.0,2.0},
		{0.0,0.0,1.0}} /*iTj*/ };


void simulate_wall()
{
	int i;
	double c1, s1, acc[2], alpha;

	//printf("Simulation wall\n");

	acc[X] =0.0 ;// (wall.ext_force[X]/wall.mass - GRAVITY);
	acc[Y] =0.0 ;// wall.ext_force[Y]/wall.mass;

	//printf("acc[X]=%f acc[Y]= %f ", acc[X], acc[Y]);

	wall.velocity[X] += 0.0; //acc[X] * DT;
	wall.velocity[Y] += 0.0; //acc[Y] * DT;

	wall.centroid[X] += 0.00; //wall.velocity[X]*DT-0.5*GRAVITY*SQR(DT);
	wall.centroid[Y] += 0.00; //wall.velocity[Y]*DT;

	wall.vertices[0][X] = wall.centroid[X] - W_WALL*0.5;
	wall.vertices[0][Y] = wall.centroid[Y] + H_WALL*0.5;

	wall.vertices[1][X] = wall.centroid[X] + W_WALL*0.5;
	wall.vertices[1][Y] = wall.centroid[Y] + H_WALL*0.5;

	wall.vertices[2][X] = wall.centroid[X] + W_WALL*0.5;
	wall.vertices[2][Y] = wall.centroid[Y] - H_WALL*0.5;

	wall.vertices[3][X] = wall.centroid[X] - W_WALL*0.5;
	wall.vertices[3][Y] = wall.centroid[Y] - H_WALL*0.5;

/*	if ((wall.centroid[X] < (MIN_X + R_OBJ)) && (wall.velocity[X] < 0.0))
		wall.velocity[X] *= -1.0;                                               
	if ((wall.centroid[X] > (MAX_X - R_OBJ)) && (wall.velocity[X] > 0.0))
		wall.velocity[X] *= -1.0;                                               
	if ((wall.centroid[Y] < (MIN_Y + R_OBJ)) && (wall.velocity[Y] < 0.0))
		wall.velocity[Y] *= -1.0;                                               
	if ((wall.centroid[Y] > (MAX_Y - R_OBJ)) && (wall.velocity[Y] > 0.0))
		wall.velocity[Y] *= -1.0; 
*/
}

