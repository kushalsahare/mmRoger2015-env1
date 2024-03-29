#include<stdio.h>
#include "geometry.h"
#include <math.h>
#include "Roger.h"


double distance(point a, point b)
{
    int i;          /* counter */
    double d=0.0;       /* accumulated distance */

    for (i=0; i<DIMENSION; i++)
        d = d + (a[i]-b[i]) * (a[i]-b[i]);

    return( sqrt(d) );
}

double InvSqrt(double x){
    double xhalf = 0.5f * x;
    int i = *(int *) &x ;
    i = 0x5f3759df - (i >> 1);
    x = *(float*)&i;
    x = x*(1.5f - xhalf*x*x);

    return x;

}
void intersect_circle_poly(circle circ, rect r, double f[2]){


    int i = 0 ;
    int j = 0 ;

    point A, B, E, F, G;

    double R = circ.r; // radius
    point C; //center
    C[X] = circ.c[X];
    C[Y] = circ.c[Y];
    //printf("Center :(%f %f)\n ",C[X], C[Y]);
    double dx, dy, dr, D, discmt, inv_dr, dt;
    double inv_len; // 1/ L(P1-P2)
    int sign_dy;

    double t; // 0 <= t <=1
    double lAB;
    double lEC;

    double d;
    double Ex, Ey, mag;

    for (i =0;  i < 4 ; i++){

        A[X] = r[i%4][X];
        A[Y]=  r[i%4][Y];
        //printf("%d (%f %f) ",(i%4), A[X], A[Y]);

        B[X] = r[(i+1)%4][X];
        B[Y]=  r[(i+1)%4][Y];
        //printf("%d (%f %f) ",(i+1)%4, B[X], B[Y]);
        
        lAB = sqrt( (B[X]-A[X])*(B[X]-A[X]) + (B[Y]-A[Y])*(B[Y]-A[Y]) );

        dx = (B[X] - A[X]) / lAB;

        dy = (B[Y] - A[Y]) / lAB;

        t = dx*(C[X]-A[X]) + dy*(C[Y]-A[Y]);

        E[X] = t*dx+A[X];
        E[Y] = t*dy+A[Y];
        //printf(" (%f %f) ", E[X], E[Y]);

        lEC = sqrt( (E[X]-C[X])*( E[X] -C[X]) +(E[Y]-C[Y])*(E[Y]-C[Y]) ); 
        
        //printf("  %f %f \n", lEC , R);

        if( lEC < R )
        {
            // compute distance from t to circle intersection point
            dt = sqrt( R*R - lEC*lEC);

            d= R - lEC ;
            // compute first intersection point
            F[X] = (t-dt)*dx + A[X];
            F[Y] = (t-dt)*dy + A[Y];

            if(t-dt >0 && t-dt < 1){
                f[X]  += K_COLLIDE* d * (E[X]-C[X]);
                f[Y]  += K_COLLIDE* d * (E[Y]-C[Y]);
          //      printf("%f\n", d);
            }
     //       printf("First point ( %f, %f) \n", F[X], F[Y]);

            // compute second intersection point
            G[X] = (t+dt)*dx + A[X];
            G[Y] = (t+dt)*dy + A[Y];
      //      printf("Second point ( %f, %f) \n", G[X], G[Y]);

            if(t+dt > 0 && t+dt < 1){
                f[X]  += K_COLLIDE* d * (E[X]-C[X]);
                f[Y]  += K_COLLIDE* d * (E[Y]-C[Y]);
            //    printf("%f\n", d);
            }
        }
       // printf("Force ( %f, %f) \n", f[X], f[Y]);
    }
}

points_to_line(point p1, point p2, line *l)
{
    if (p1[X] == p2[X]) {
        l->a = 1;
        l->b = 0;
        l->c = -p1[X];
    } else {
        l->b = 1;
        l->a = -(p1[Y]-p2[Y])/(p1[X]-p2[X]);
        l->c = -(l->a * p1[X]) - (l->b * p1[Y]);
    }
}

point_and_slope_to_line(point p, double m, line *l)
{
    l->a = -m;
    l->b = 1;
    l->c = -((l->a*p[X]) + (l->b*p[Y]));
}

int parallelQ(line l1, line l2)
{
    return ( (fabs(l1.a-l2.a) <= EPSILON) &&
            (fabs(l1.b-l2.b) <= EPSILON) );
}

int same_lineQ(line l1, line l2)
{
    return ( parallelQ(l1,l2) && (fabs(l1.c-l2.c) <= EPSILON) );
}


intersection_point(line l1, line l2, point p)
{
    if (same_lineQ(l1,l2)) {
        printf("Warning: Identical lines, all points intersect.\n");
        p[X] = p[Y] = 0.0;
        return;
    }

    if (parallelQ(l1,l2) == TRUE) {
        printf("Error: Distinct parallel lines do not intersect.\n");
        return;
    }

    p[X] = (l2.b*l1.c - l1.b*l2.c) / (l2.a*l1.b - l1.a*l2.b);

    if (fabs(l1.b) > EPSILON)   /* test for vertical line */
        p[Y] = - (l1.a * (p[X]) + l1.c) / l1.b;
    else
        p[Y] = - (l2.a * (p[X]) + l2.c) / l2.b;
}

closest_point(point p_in, line l, point p_c)
{
    line perp;      /* perpendicular to l through (x,y) */

    if (fabs(l.b) <= EPSILON) { /* vertical line */
        p_c[X] = -(l.c);
        p_c[Y] = p_in[Y];
        return;
    }

    if (fabs(l.a) <= EPSILON) { /* horizontal line */
        p_c[X] = p_in[X];
        p_c[Y] = -(l.c);
        return;
    }

    point_and_slope_to_line(p_in,1/l.a,&perp); /* non-degenerate line */
    /*printf("perpendicular bisector "); print_line(perp);*/
    intersection_point(l,perp,p_c);
    /*printf("closest point "); print_point(p_c);*/
}

/***********************************************************************/

copy_point(point a, point b)
{
    int i;          /* counter */

    for (i=0; i<DIMENSION; i++) b[i] = a[i];
}

swap_point(point a, point b)
{
    point c;        /* temporary point */

    copy_point(a,c);
    copy_point(b,a);
    copy_point(c,b);
}


points_to_segment(point a, point b, segment *s)
{
    copy_point(a,s->p1);
    copy_point(b,s->p2);
}

segment_to_points(segment s, point p1, point p2)
{
    copy_point(s.p1,p1);
    copy_point(s.p2,p2);
}


int point_in_box(point p, point b1, point b2)
{
    return( (p[X] >= MIN(b1[X],b2[X])) && (p[X] <= MAX(b1[X],b2[X]))
            && (p[Y] >= MIN(b1[Y],b2[Y])) && (p[Y] <= MAX(b1[Y],b2[Y])) );
}

int segments_intersect(segment s1, segment s2)
{
    line l1,l2;     /* lines containing the input segments */
    point p;        /* intersection point */

    points_to_line(s1.p1,s1.p2,&l1);
    points_to_line(s2.p1,s2.p2,&l2);

    if (same_lineQ(l1,l2))  /* overlapping or disjoint segments */
        return( point_in_box(s1.p1,s2.p1,s2.p2) ||
                point_in_box(s1.p2,s2.p1,s2.p2) ||
                point_in_box(s2.p1,s1.p1,s1.p2) ||
                point_in_box(s2.p2,s1.p1,s1.p2) );

    if (parallelQ(l1,l2)) return(FALSE);

    intersection_point(l1,l2,p);

    return( point_in_box(p,s1.p1,s1.p2) && point_in_box(p,s2.p1,s2.p2) );
}

double signed_triangle_area(point a, point b, point c)
{
    return( (a[X]*b[Y] - a[Y]*b[X] + a[Y]*c[X]
                - a[X]*c[Y] + b[X]*c[Y] - c[X]*b[Y]) / 2.0 );
}

double triangle_area(point a, point b, point c)
{
    return( fabs(signed_triangle_area(a,b,c)) );
}

int ccw(point a, point b, point c)
{
    double signed_triangle_area();

    return (signed_triangle_area(a,b,c) > EPSILON);
}

int cw(point a, point b, point c)
{
    double signed_triangle_area();

    return (signed_triangle_area(a,b,c) < - EPSILON);
}

int collinear(point a, point b, point c)
{
    double signed_triangle_area();

    return (fabs(signed_triangle_area(a,b,c)) <= EPSILON);
}

print_points(point p[], int n)
{
    int i;                  /* counter */

    for (i=0; i<n; i++)
        printf("(%lf,%lf)\n",p[i][X],p[i][Y]);
}

print_polygon(polygon *p)
{
    int i;          /* counter */

    for (i=0; i<p->n; i++)
        printf("(%lf,%lf)\n",p->p[i][X],p->p[i][Y]);
}

print_point(point p)
{
    printf("%7.3lf %7.3lf\n",p[X],p[Y]);
}

print_line(line l)
{
    printf("(a=%7.3lf,b=%7.3lf,c=%7.3lf)\n",l.a,l.b,l.c);
}

print_segment(segment s)
{
    printf("segment: ");
    print_point(s.p1);
    print_point(s.p2);
}


double cross(point v, point w){

    return v[X]*w[Y] - v[Y]*w[X];

}

double dot(point v, point w){

    return v[X]*w[X] + v[Y]*w[X];
}
/**************************************/
