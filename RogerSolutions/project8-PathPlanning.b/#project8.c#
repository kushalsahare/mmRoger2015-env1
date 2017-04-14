/*************************************************************************/
/* File:        project8.c                                               */
/* Description: User project #8 - harmonic function code                 */
/* Author:      Rod Grupen                                               */
/* Date:        12-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

/************************************************************************/
/************************************************************************/

double step_size;

void sor(roger)
Robot * roger;
{
  int i, j, sor_count=0, converged = FALSE;
  double sor_once();

  while (!converged && (sor_count < 5000)) {
    ++sor_count;
    if (sor_once(roger) < THRESHOLD) converged = TRUE;
  }
  if (sor_count > 1)
    printf("completed harmonic function --- %d iterations\n", sor_count);
}

// one complete backup, only dirichlet boundary conditions
double sor_once(roger)
Robot * roger;
{
  int i, j, ipos, ineg, jpos, jneg;
  double residual, max, front, back, up, down;

  // iterate over entire map once
  // return the  maximum change in the potential value over the entire
  // occupancy map as a means of determining convergence
  max = 0.0;
  for (i = 0; i < NBINS; ++i) {
    ipos = (i + 1) % NBINS;
    ineg = (i - 1 + NBINS) % NBINS;
    for (j = 0; j < NBINS; ++j) {
      jpos = (j + 1) % NBINS;
      jneg = (j - 1 + NBINS) % NBINS;
      if (roger->world_map.occupancy_map[i][j] == FREESPACE) {
	up = roger->world_map.potential_map[i][jpos];
	down = roger->world_map.potential_map[i][jneg];
	front = roger->world_map.potential_map[ipos][j];
	back = roger->world_map.potential_map[ineg][j];
	
	residual = front + back + up + down 
	  - 4.0 * roger->world_map.potential_map[i][j];
	roger->world_map.potential_map[i][j] += 0.48 * residual;
	// printf("SOR:  potential(%d,%d)=%f\n", 
	//        i,j,roger->world_map.potential_map[i][j]);
	if (fabs(residual) > max) max = fabs(residual);
      }
    }
  }
  return(max);
}

// corrected version 10-6-09
double compute_gradient(x, y, roger, grad)
double x, y;
Robot * roger;
double grad[2]; // grad = [ d(phi)/dx  d(phi)/dy ]
{
  int i0,i1,j0,j1;
  double mag, dphi_di, dphi_dj, del_x, del_y;

  j0 = (int) ((x-MIN_X)/XDELTA);
  j1 = (j0+1);
  i1 = NBINS - (int) ((y - MIN_Y)/YDELTA); // (int) ((MAX_Y - y)/YDELTA);?
  i0 = (i1-1);

  del_x = (x-MIN_X)/XDELTA - j0;
  del_y = (NBINS - (y - MIN_Y)/YDELTA) - i0;
  
  dphi_dj = ((1.0-del_y)*(roger->world_map.potential_map[i0][j1] 
			  - roger->world_map.potential_map[i0][j0] ) +
	     (del_y)*(roger->world_map.potential_map[i1][j1] 
		      - roger->world_map.potential_map[i1][j0]  ) );
  dphi_di = ((1.0-del_x)*(roger->world_map.potential_map[i1][j0] 
			  - roger->world_map.potential_map[i0][j0] ) +
	     (del_x)*(roger->world_map.potential_map[i1][j1] 
		      - roger->world_map.potential_map[i0][j1]  ) );
  
  grad[0] = dphi_dj; grad[1] = -dphi_di;

  mag = sqrt(SQR(grad[0])+SQR(grad[1]));

  if (mag>THRESHOLD) {
    grad[0] /= mag; grad[1] /= mag;
  }
  else grad[0] = grad[1] = 0;
  return(mag);
}

void follow_path(roger)
Robot *roger;
{
  int xbin, ybin;
  double x, y, bb, mag, grad[2], compute_gradient();
  double next_stepsize, velocity_control();

  x = roger->base_position[X];
  y = roger->base_position[Y];

  ybin = (int)((MAX_Y - y)/YDELTA);
  xbin = (int)((x - MIN_X)/XDELTA);

  grad[X] = grad[Y] = 0.0;
  if (roger->world_map.occupancy_map[ybin][xbin]!=GOAL) {
    mag = compute_gradient(x, y, roger, grad);
    roger->base_setpoint[X] = x - step_size*grad[X];
    roger->base_setpoint[Y] = y - step_size*grad[Y];
    roger->base_setpoint[THETA] = atan2(-grad[Y], -grad[X]);
  }
  else { // current bin is a GOAL
    //  roger->base_setpoint[X] = x;
    //    roger->base_setpoint[Y] = y;
    //    roger->base_setpoint[THETA] = roger->base_position[THETA];
    roger->world_map.color_map[ybin][xbin] = NOFILL;
    roger->world_map.occupancy_map[ybin][xbin] = FREESPACE;
    printf("AT GOAL BIN x %i , y %i  \n" ,xbin , ybin);
  }
}

/************************************************************************/
/************************************************************************/

void sor(), follow_path();

void project8_control(roger, time)
Robot *roger;
double time;
{ 
  double velocity_control();

  sor(roger);
  step_size = velocity_control(roger);
  follow_path(roger);
}

/************************************************************************/
void project8_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project8_enter_params() 
{
  printf("Project 8 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project8_visualize(roger)
Robot* roger;
{
  void draw_streamlines(), draw_current_streamline();
  //  draw_streamlines();
  draw_current_streamline();
}

