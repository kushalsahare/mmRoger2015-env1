/**************************************************************************/
/* File:        roger.h                                                   */
/* Description: all the compile time constants that define Roger          */
/*              IMPORTANT --- READ ONLY --- DO NOT EDIT THIS FILE         */
/* Author:      Rod Grupen                                                */
/* Date:        11-1-2009                                                 */
/**************************************************************************/

#ifndef ROGER_H_
#define ROGER_H_
/**************************************************************************/
// environmental constants
/**************************************************************************/
#define GRAVITY        0.0   /* [m/sec^2] - gravitational constant -y dir */
                             /* mm sim does not yet accomodate gravity    */

/**************************************************************************/
// morphological constants
/**************************************************************************/
#define NDOF           8
#define NWHEELS        2
#define NARMS          2
#define NARM_LINKS     2
#define NARM_JOINTS    2
#define NARM_FRAMES    4
#define NEYES          2
#define NPIXELS      128     /* array size of 1D image structure */

/**************************************************************************/
// geometrical and inertial constants
/**************************************************************************/
#define M_EYE          0.05  /* [kg] - mass of the eye */
#define L_EYE          0.04  /* [m] - the length of an eye link*/
#define I_EYE  (M_EYE*L_EYE*L_EYE) /* [kg m^2] - eye moment of inertia */
#define BASELINE       0.08  /* [m] - 1/2 distance between eyes */
#define FOCAL_LENGTH  64.0   /* [pixels] - focal length */

#define M_ARM1         0.2   /* [kg] - mass of arm link 1 (upper arm) */
#define M_ARM2         0.2   /* [kg] - mass of arm link 2 (forearm)   */
#define ARM_OFFSET     0.18  /* [m] - 1/2 distance between arms */
#define L_ARM1         0.5   /* [m] - the length of link 1 */
#define L_ARM2         0.5   /* [m] - the length of link 2 */
#define I_ARM1 (M_ARM1*L_ARM1*L_ARM1) /* [kg m^2] - link 1 moment of inertia */
#define I_ARM2 (M_ARM2*L_ARM2*L_ARM2) /* [kg m^2] - link 2 moment of inertia */

#define M_BASE         1.0   /* [kg] */
#define R_BASE         0.16  /* [m] - the radius of Roger's body */
#define I_BASE (M_BASE*R_BASE*R_BASE) /* [kg m^2] */
#define R_AXLE         0.20  /* [m] - the radius to wheel contact w/ground */
#define R_WHEEL        0.1   /* [m] - the radius of the wheel */

#define M_WBODY (M_BASE + 2.0*M_EYE + 2.0*(M_ARM1+M_ARM2)) /* [kg] */
#define I_WBODY (M_WBODY*R_BASE*R_BASE)

// Parameters for inertial objects that can collide in the Cartesian plane
// #define NBODY 5 // 0:base, 1:arm1, 2:arm2, 3:toy, 4:occ. grid
enum iobj {
  BASE = 0,
  ARM1,
  ARM2,
  TOY,
  OGRID,
  NBODY
};

#define USE_SPHERE     0

#define CIRCLE         0      /* object "shape" identifier */
#define TRIANGLE       1 
#define RECT           2

#define M_BALL       0.5   /* [kg] */
#define I_BALL       0.02  /* [kg m^2] */
#define R_BALL       0.20  /* [m] - the radius of the red ball */

#define M_TRIANGLE   0.5   /* [kg] = (3*mass) of the balls at the vertices */
#define I_TRIANGLE   0.02  /* [kg m^2] */
#define R_VERTEX     0.15  /* [m] - the radius of the VERTEX ball */
#define R_SPOKE      0.15  /* [m] - "spoke" length */

#define H_RECT       0.25  /* [m]  - height of the rect */
#define W_RECT       0.25  /* [m]  - width of the rect */
#define M_RECT       0.50  /* [kg] - mass of the rect */
#define I_RECT       0.02  /* [kg m^2] */

#define X            0
#define Y            1
#define TRUE         1
#define FALSE        0

/**************************************************************************/
// graphical constants - just for rendering
/**************************************************************************/
#define WHEEL_THICKNESS 0.04  /* [m] - the thickness of a wheel */

#define R_JOINT        0.02  /* [m] - the radius of an arm joint */
#define R_TACTILE      0.08  /* [m] - radius of tactile cell */

#define R_EYE          0.07  /* [m] - the radius of an eye */
#define R_PUPIL        0.04  /* [m] - the radius of a pupil */

#define IMAGE_WIDTH  256
#define FOV (atan2((double)(NPIXELS/2.0), (double)FOCAL_LENGTH))
                             /* [rad] - half angle of FOV  */

// parameters used to compute collision forces between the arm and the ball
//#define K_COLLIDE    500.0
//#define B_COLLIDE      2.5
//#define OBJ_GRAVITY    0.0  /* neutrally buoyant */

#define K_COLLIDE    45500.0
#define B_COLLIDE      5.5
#define OBJ_GRAVITY    0.0  /* neutrally buoyant */

// probably should go in control.h
#define VISCOSITY      0.05
#define STATIC_FORCE_THRESHOLD 10.0
#define MU			0.5 /* coeff. of friction */

// for visual appearance of ball - probably should go in control.h
#define NFEATURES      3
#define CENTROID       0
#define LEFT_EDGE      1
#define RIGHT_EDGE     2

/**************************************************************************/
// motor constants
/**************************************************************************/
// stall-torques
#define WHEEL_TS		475.0
#define EYE_TS 			  2.7
#define ELBOW_TS		120.7
#define SHOULDER_TS             205.3

// motor no-load speed
#define WHEEL_W0		175.0
#define EYE_W0 			122.2
#define ELBOW_W0		 50.3
#define SHOULDER_W0              30.1

#endif //ROGER_H_
