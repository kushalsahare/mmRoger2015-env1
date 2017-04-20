/*************************************************************************/
/* File:        project12.c                                               */
/* Description: User project #12 - empty project directory for project    */
/*              development                                             */
/* Date:        03-30-2013                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

void project12_control(roger, time)
Robot* roger;
double time;
{ }

/************************************************************************/
void project12_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project12_enter_params() 
{
  printf("Project 12 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project12_visualize(roger)
Robot* roger;
{ }
