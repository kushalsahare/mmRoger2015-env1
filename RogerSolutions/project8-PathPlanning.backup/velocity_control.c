/*************************************************************************/
/* File:        velocity_control.c                                       */
/* Description: Velocity controlled path traversal                       */
/* Author:      Mitchell Hebert                                          */
/* Date:        01-20-2015                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

#define VELOCITY_CONTROL    TRUE

double velocity_control(default_step, roger)
double default_step;
Robot *roger;
{
  if (VELOCITY_CONTROL) { 
    return(4.0*default_step);
  }
  else{
    return(default_step);
  }
}
