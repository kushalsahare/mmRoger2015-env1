/*************************************************************************/
/* File:        RRTPlanning.c                               */
/* Description:                                                          */
/* Date:        01-29-2011                                               */
/*************************************************************************/

#include <math.h>
#include <stdio.h>
#include <time.h>
#include "Xkw/Xkw.h"

#include "Roger.h"
#include "simulate.h"
#include "control.h"

#define REACHED 0
#define ADVANCED 1
#define TRAPPED 2
#define PI 3.14159265
#define LARGE_VAL 10000000

int num_vertices, num_edges;
double goal[7];
int path[NSTEPS];
int path_index;
int where_in_path = 0;
Boolean got_plan = FALSE;

int add_vertex(q)
double *q;
{
	int i;
	for(i=0; i<7; i++)
		v[num_vertices].q[i] = q[i];
	num_vertices += 1;
	
	return num_vertices;
}

int add_edge(v1, v2, edge_dist)
int v1,v2;
double edge_dist;
{
	double get_edge_distance();
	e[num_edges].v1 = v1;
	e[num_edges].v2 = v2;
	edge_distance[v1][v2] = get_edge_distance(v[v1].q, v[v2].q);
	edge_distance[v2][v1] = edge_distance[v1][v2];
	//next[v1][v2] = v2;
	//next[v2][v1] = v1;
	num_edges += 1;
	
	return num_edges;
}


int draw_doorway(roger)
Robot * roger;
{
	int xbin, ybin;
	ybin = NBINS/2 + 10;
	
	for (xbin=0; xbin<((MAX_X-MIN_X)/(2*XDELTA))-7; xbin++)
	{
		//printf("Doorway xbin : %d, ybin : %d\n",xbin,ybin);
		/*	roger->world_map.occupancy_map[ybin-1][xbin] = OBSTACLE;
		 roger->world_map.potential_map[ybin-1][xbin] = 1.0;
		 roger->world_map.color_map[ybin-1][xbin] = DARKYELLOW;
		 */	
		roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
		roger->world_map.potential_map[ybin][xbin] = 1.0;
		roger->world_map.color_map[ybin][xbin] = DARKYELLOW;
		/*	
		 roger->world_map.occupancy_map[ybin+1][xbin] = OBSTACLE;
		 roger->world_map.potential_map[ybin+1][xbin] = 1.0;
		 roger->world_map.color_map[ybin+1][xbin] = DARKYELLOW;
		 */
	}
	
	for (xbin=((MAX_X-MIN_X)/(2*XDELTA))+7; xbin<(MAX_X-MIN_X)/XDELTA; xbin++)
	{
		//printf("doorway xbin : %d, ybin : %d\n",xbin,ybin);
		/*	roger->world_map.occupancy_map[ybin-1][xbin] = OBSTACLE;
		 roger->world_map.potential_map[ybin-1][xbin] = 1.0;
		 roger->world_map.color_map[ybin-1][xbin] = DARKYELLOW;
		 */	
		roger->world_map.occupancy_map[ybin][xbin] = OBSTACLE;
		roger->world_map.potential_map[ybin][xbin] = 1.0;
		roger->world_map.color_map[ybin][xbin] = DARKYELLOW;
		
		/*	roger->world_map.occupancy_map[ybin+1][xbin] = OBSTACLE;
		 roger->world_map.potential_map[ybin+1][xbin] = 1.0;
		 roger->world_map.color_map[ybin+1][xbin] = DARKYELLOW;	
		 */
	}
}


Boolean robot_in_collision(roger, q)
Robot * roger;
double *q;
{
	int DELTA = 2;
	double LDELTA = 0.01;
	double x,y;
	int xbin, ybin, i;
	Boolean in_collision = FALSE;
	double base_position[3], arm_theta[2][2];
	double wTb[4][4], ref_b[4], elbow_w[4], end_eff_w[4], slope;
	
	base_position[0] = q[0];
	base_position[1] = q[1];
	base_position[2] = q[2];
	arm_theta[LEFT][0] = q[3];
	arm_theta[LEFT][1] = q[4];
	arm_theta[RIGHT][0] = q[5];
	arm_theta[RIGHT][1] = q[6];
	//printf("\n Robot position : %f, %f",base_position[0],base_position[1]);
	//printf("\n Left arm : %f,%f",arm_theta[LEFT][0],arm_theta[LEFT][1]);
	//printf("\n Right arm : %f,%f\n",arm_theta[RIGHT][0],arm_theta[RIGHT][1]);
	
	/*
	 for(i=0; i<7; i++)
	 printf("\t q[%d] : %lf,",i,q[i]);
	 printf("\n");
	 */
	
	//Check if the base is in collision
	for (i=0; i<360/DELTA; i++)
	{
		x = base_position[0] + R_BASE*cos(i*DELTA*2*PI/360);
		y = base_position[1] + R_BASE*sin(i*DELTA*2*PI/360);
		
		xbin = (x - MIN_X) / XDELTA;
		ybin = NBINS - (y - MIN_Y) / YDELTA;
		
		//printf("\n xbin : %d, ybin : %d",xbin, ybin);
		if(roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			//printf("\n Robot base is in collision \n");
			in_collision = TRUE;
			break;
		}
	}
	
	if (in_collision == TRUE)
		return in_collision;
	
	//Check if the left arm is in collision
	construct_wTb(base_position, wTb);
	
	//Get position of left elbow
	ref_b[0]=L1*cos(arm_theta[LEFT][0]);
	ref_b[1]=R_BASE+L1*sin(arm_theta[LEFT][0]);
	ref_b[2] = 0.0;
	ref_b[3] = 1.0;
	matXvec(wTb, ref_b, elbow_w);
	//printf("\n Elbow position of left arm : %f,%f",elbow_w[0], elbow_w[1]);
	
	slope = atan2(elbow_w[1]-base_position[1], elbow_w[0]-base_position[0]);
	for(i=0; i<(int)(L1/LDELTA); i++)
	{
		x = base_position[0] + i*LDELTA*cos(slope);
		y = base_position[1] + i*LDELTA*sin(slope);
		
		xbin = (x - MIN_X) / XDELTA;
		ybin = NBINS - (y - MIN_Y) / YDELTA;
		
		//printf("\n xbin : %d, ybin : %d",xbin, ybin);
		if(roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			//printf("\n Robot to left elbow is in collision \n");
			in_collision = TRUE;
			break;
		}
	}
	
	if (in_collision == TRUE)
		return in_collision;	
	
	//Get position of left end-effector
	ref_b[0]=L1*cos(arm_theta[LEFT][0]) + L2*cos(arm_theta[LEFT][0]+arm_theta[LEFT][1]);
	ref_b[1]=R_BASE+L1*sin(arm_theta[LEFT][0]) + L2*sin(arm_theta[LEFT][0]+arm_theta[LEFT][1]);
	ref_b[2] = 0.0;
	ref_b[3] = 1.0;
	matXvec(wTb, ref_b, end_eff_w);
	//printf("\n End effector position of left arm : %f,%f",end_eff_w[0], end_eff_w[1]);
	
	slope = atan2(end_eff_w[1]-elbow_w[1], end_eff_w[0]-elbow_w[0]);
	for(i=0; i<(int)(L2/LDELTA); i++)
	{
		x = elbow_w[0] + i*LDELTA*cos(slope);
		y = elbow_w[1] + i*LDELTA*sin(slope);
		
		xbin = (x - MIN_X) / XDELTA;
		ybin = NBINS - (y - MIN_Y) / YDELTA;
		
		//printf("\n xbin : %d, ybin : %d",xbin, ybin);
		if(roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			//printf("\n Left Elbow to end-effector is in collision \n");
			in_collision = TRUE;
			break;
		}
	}
	
	if (in_collision == TRUE)
		return in_collision;	
	
	//Check if the right arm is in collision
	construct_wTb(base_position, wTb);
	
	//Get position of right elbow
	ref_b[0]=L1*cos(arm_theta[RIGHT][0]);
	ref_b[1]=-R_BASE+L1*sin(arm_theta[RIGHT][0]);
	ref_b[2] = 0.0;
	ref_b[3] = 1.0;
	matXvec(wTb, ref_b, elbow_w);
	//printf("\n Elbow position of right arm : %f,%f",elbow_w[0], elbow_w[1]);
	
	slope = atan2(elbow_w[1]-base_position[1], elbow_w[0]-base_position[0]);
	for(i=0; i<(int)(L1/LDELTA); i++)
	{
		x = base_position[0] + i*LDELTA*cos(slope);
		y = base_position[1] + i*LDELTA*sin(slope);
		
		xbin = (x - MIN_X) / XDELTA;
		ybin = NBINS - (y - MIN_Y) / YDELTA;
		
		//printf("\n xbin : %d, ybin : %d",xbin, ybin);
		if(roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			//printf("\n Robot to right elbow is in collision \n");
			in_collision = TRUE;
			break;
		}
	}
	
	if (in_collision == TRUE)
		return in_collision;	
	
	//Get position of right end-effector
	ref_b[0]=L1*cos(arm_theta[RIGHT][0]) + L2*cos(arm_theta[RIGHT][0]+arm_theta[RIGHT][1]);
	ref_b[1]=-R_BASE+L1*sin(arm_theta[RIGHT][0]) + L2*sin(arm_theta[RIGHT][0]+arm_theta[RIGHT][1]);
	ref_b[2] = 0.0;
	ref_b[3] = 1.0;
	matXvec(wTb, ref_b, end_eff_w);
	//printf("\n End effector position of right arm : %f,%f",end_eff_w[0], end_eff_w[1]);
	
	slope = atan2(end_eff_w[1]-elbow_w[1], end_eff_w[0]-elbow_w[0]);
	for(i=0; i<(int)(L2/LDELTA); i++)
	{
		x = elbow_w[0] + i*LDELTA*cos(slope);
		y = elbow_w[1] + i*LDELTA*sin(slope);
		
		xbin = (x - MIN_X) / XDELTA;
		ybin = NBINS - (y - MIN_Y) / YDELTA;
		
		//printf("\n xbin : %d, ybin : %d",xbin, ybin);
		if(roger->world_map.occupancy_map[ybin][xbin] == OBSTACLE)
		{
			//printf("\n Right Elbow to end-effector is in collision \n");
			in_collision = TRUE;
			break;
		}
	}
	
	if (in_collision == TRUE)
		return in_collision;	
	
	return in_collision;
}

void random_config(double *q)
{
	int i,j;
	
	for(i=0; i<7; i++)
	{
		//unsigned int iseed = (unsigned int)time(NULL);
		//srand (iseed);
		j = (int)( rand() % NBINS + 1 );
		
		if (i==0)
			q[i] = MIN_X + j*(MAX_X-MIN_X)/NBINS;
		else if(i==1)
			q[i] = MIN_Y + j*(MAX_Y-MIN_Y)/NBINS;
		//else if(i>=2)
		//	q[i] = j*2*PI/(NBINS);
		else if(i>=2)
			q[i] = -PI + j*2*PI/NBINS;
	}
	
}

double get_edge_distance(q1, q2)
double *q1, *q2;
{
	double dist_base_pos, dist_base_theta, dist_larm, dist_rarm;
	double total_distance;
	
	//Compute distance in base config
	dist_base_pos = sqrt(pow(q1[0]-q2[0],2)+pow(q1[1]-q2[1],2));
	dist_base_theta = fabs(q1[2]-q2[2]);
	dist_larm = sqrt(pow(q1[3]-q2[3],2)+pow(q1[4]-q2[4],2));
	dist_rarm = sqrt(pow(q1[5]-q2[5],2)+pow(q1[6]-q2[6],2));
	
	//Weight the distances - Giving equal weights to all
	total_distance = 0.6*dist_base_pos + 0.1*dist_base_theta + 0.15*dist_larm + 0.15*dist_rarm;
	
	return total_distance;
}


int nearest_neighbor(q, q_near)
double *q, *q_near;
{
	int i, min_index = 0;
	double dist_base_pos, dist_base_theta, dist_larm, dist_rarm;
	double total_distance, min_dist = 10000;
	for(i=0; i<num_vertices; i++)
	{
		total_distance = get_edge_distance(q, v[i].q);
		//printf("td : %lf",total_distance);
		//Compute distance in base config
		/*
		 dist_base_pos = sqrt(pow(q[0]-v[i].q[0],2)+pow(q[1]-v[i].q[1],2));
		 dist_base_theta = fabs(q[2]-v[i].q[2]);
		 dist_larm = sqrt(pow(q[3]-v[i].q[3],2)+pow(q[4]-v[i].q[4],2));
		 dist_rarm = sqrt(pow(q[5]-v[i].q[5],2)+pow(q[6]-v[i].q[6],2));
		 
		 //Weight the distances - Giving equal weights to all
		 total_distance = 0.6*dist_base_pos + 0.1*dist_base_theta + 0.15*dist_larm + 0.15*dist_rarm;
		 */	
		if(total_distance < min_dist)
		{
			min_dist = total_distance;
			min_index = i;
		}
	}
	
	for(i=0; i<7; i++)
		q_near[i] = v[min_index].q[i];
	
	return min_index;
}

int nearest_neighbor_cartesian(q, q_near)
double *q, *q_near;
{
	int i, min_index = 0;
	double dist_base_pos, dist_base_theta, dist_larm, dist_rarm;
	double total_distance, min_dist = 10000;
	for(i=0; i<num_vertices; i++)
	{
		
		total_distance = sqrt(pow(q[0]-v[i].q[0],2)+pow(q[1]-v[i].q[1],2));
		
		if(total_distance < min_dist)
		{
			min_dist = total_distance;
			min_index = i;
		}
	}
	
	for(i=0; i<7; i++)
		q_near[i] = v[min_index].q[i];
	
	return min_index;
}


int new_config(roger, q, q_near, q_new)
Robot * roger;
double *q, *q_near, *q_new;
{
	int i,j;
	int NUMSEARCHDELTA = 1;
	int MAXDELTA = 20;
	double q_old[7];
	Boolean in_collision;
	while (TRUE)
	{
		for(i=0; i<7; i++)
		{
			q_new[i] = q_near[i] + NUMSEARCHDELTA*(q[i]-q_near[i])/(MAXDELTA);
		}
		in_collision = robot_in_collision(roger, q_new);
		if(in_collision == FALSE)
			NUMSEARCHDELTA += 1;
		else
		{
			if (NUMSEARCHDELTA > 1) 
			{
				for(j=0; j<7; j++)
					q_new[j] = q_old[j];
				
				return ADVANCED;
			}
			else
				return TRAPPED;
		}
		
		if (NUMSEARCHDELTA >= MAXDELTA)
		{
			return REACHED;
		}
		for(j=0; j<7; j++)
			q_old[j] = q_new[j];
		
	}
}

void floyd_warshall_old() {
	int i, j, k;
	for (k = 0; k < num_vertices; k++) {
		for (i = 0; i < num_vertices; i++)
			for (j = 0; j < num_vertices; j++)
			/* If i and j are different nodes and if 
			 the paths between i and k and between
			 k and j exist, do */
				if (( edge_distance[i][k] * edge_distance[k][j] != 0) && (i != j))
				/* See if you can't get a shorter path
				 between i and j by interspacing
				 k somewhere along the current
				 path */
					if (((edge_distance[i][k] + edge_distance[k][j] < edge_distance[i][j]) && (i!=k) && (j!=k)) ||
						(edge_distance[i][j] == 0))
					{
						edge_distance[i][j] = edge_distance[i][k] + edge_distance[k][j];
						next[i][j] = k;	
					}
	}
	
	printf("Print shotest path \n:");
	for(i=0; i<num_vertices; i++)
		for(j=0; j<num_vertices; j++)
			if(edge_distance[i][j] < 100000)
				printf("i : %d, j : %d  -> %lf\n",i,j,edge_distance[i][j]);
}

void floyd_warshall() 
{
	int i, j, k;
	for (k = 0; k < num_vertices; ++k) {
		for (i = 0; i < num_vertices; ++i)
			for (j = 0; j < num_vertices; ++j)
			{
				if (i==k || j==k)
					break;
				if ((edge_distance[i][k] + edge_distance[k][j] < edge_distance[i][j]))
				{
					edge_distance[i][j] = edge_distance[i][k]+edge_distance[k][j];
					next[i][j] = k;
					next[j][i] = k;
				}
			}
	}
	
/*	int count = 0;
	printf("Print shotest path \n:");
	for(i=0; i<num_vertices; i++)
		for(j=0; j<num_vertices; j++)
			if(i!=j)
			{
				printf("i : %d, j : %d  -> %lf\t",i,j,edge_distance[i][j]);
				count++;
			}
	printf("Count : %d\n",count);
 */
}

int get_path(node1, node2)
int node1, node2;
{
	//printf("Inside get path : %d, %d, %d\n",node1,node2,next[node1][node2]);
	int intermediate;
	if (edge_distance[node1][node2] == LARGE_VAL)
	{
		printf("No Path \n");
		return 0;
	}
	intermediate = next[node1][node2];
	if (intermediate == -1)
	{
		path[path_index++] = node2;
		//printf("--- %lf \n",edge_distance[node1][node2]);
		return 0;
	}
	else
	{
		get_path(node1, intermediate);
		//printf(" %d \n", intermediate);
		get_path(intermediate, node2);
	}
	//printf("Path computed\n");
	return 0;
}

void build_rrt(roger)
Robot * roger;
{
	int i,j;
	//random_config();
	//Initialize the tree
	//q : BASE_X, BASE_Y, BASE_THETA, LEFT_ARM_THETA1, LEFT_ARM_THETA2, RIGHT_ARM_THETA1, RIGHT_ARM_THETA2
	double q[7], q_near[7], q_new[7], goal_near[7];
	int id1, id2, ret;
	unsigned int iseed = (unsigned int)time(NULL);
	srand (iseed);
	num_vertices = 0;
	num_edges = 0;
	//int nearest_neighbor(), new_config();
	
	q[0] = roger->base_position[0];
	q[1] = roger->base_position[1];
	q[2] = roger->base_position[2];
	q[3] = roger->arm_theta[LEFT][0];
	q[4] = roger->arm_theta[LEFT][1];
	q[5] = roger->arm_theta[RIGHT][0];
	q[6] = roger->arm_theta[RIGHT][1];
	
	add_vertex(q);
	
	//Initialize edge distance
	for(i=0; i<NSTEPS; i++)
		for(j=0; j<NSTEPS; j++)
		{
			if (i == j)
				edge_distance[i][j] = 0;
			else
				edge_distance[i][j] = LARGE_VAL;
			next[i][j] = -1;
		}
	
	for (i=0; i<NSTEPS; i++)
	{
		//printf("\t i : %d",i);
		random_config(q);
		id1 = nearest_neighbor(q,q_near);
		//printf("\t id1  : %d",id1);
		ret =new_config(roger,q,q_near,q_new);
		if (ret == REACHED || ret == ADVANCED)
		{
			id2 = add_vertex(q_new);
			//printf("\t id2 : %d",id2);
			add_edge(id1, id2-1);
		}
	}
	printf("\n RRT computed with %d edges and %d vertices\n",num_edges, num_vertices);
	floyd_warshall();
	printf("Finished computing all pairs shortest paths \n");
	//draw_rrt(v,e,num_edges);

	id1 = nearest_neighbor_cartesian(goal, goal_near);
	q[0] = roger->base_position[0];
	q[1] = roger->base_position[1];
	q[2] = roger->base_position[2];
	q[3] = roger->arm_theta[LEFT][0];
	q[4] = roger->arm_theta[LEFT][1];
	q[5] = roger->arm_theta[RIGHT][0];
	q[6] = roger->arm_theta[RIGHT][1];
	
	id2 = nearest_neighbor_cartesian(q, q_near);
	printf("id1 : %d, id2 : %d\n",id1, id2);
	get_path(id1, id2);
	for(i =path_index ; i>=0; i--)
	{
		printf("Path %d : %d \n",i,path[i]);
		printf("Goal %d : %lf,%lf,%lf,%lf,%lf,%lf,%lf\n",i,v[path[i]].q[0],v[path[i]].q[1],v[path[i]].q[2],v[path[i]].q[3],
			   v[path[i]].q[4],v[path[i]].q[5],v[path[i]].q[6]);
	}
	where_in_path = path_index;
}

void set_goal(roger, x_goal, y_goal)
Robot * roger;
double x_goal, y_goal;
{
	goal[0] = x_goal;
	goal[1] = y_goal;
	goal[2] = roger->base_position[2];
	goal[3] = roger->arm_theta[LEFT][0];
	goal[4] = roger->arm_theta[LEFT][1];
	goal[5] = roger->arm_theta[RIGHT][0];
	goal[6] = roger->arm_theta[RIGHT][1];
	/*
	goal[2] = roger->base_position[2]+PI/2;
	goal[3] = roger->arm_theta[LEFT][0]+PI/2;
	goal[4] = roger->arm_theta[LEFT][1]+PI/2;
	goal[5] = roger->arm_theta[RIGHT][0]+PI/2;
	goal[6] = roger->arm_theta[RIGHT][1]+PI/2;
	 */
}

void plan(roger)
Robot * roger;
{
int tmp;
	if(got_plan == FALSE)
		return;
	
	double distance;
	double q[7];

	q[0] = roger->base_position[0];
	q[1] = roger->base_position[1];
	q[2] = roger->base_position[2];
	q[3] = roger->arm_theta[LEFT][0];
	q[4] = roger->arm_theta[LEFT][1];
	q[5] = roger->arm_theta[RIGHT][0];
	q[6] = roger->arm_theta[RIGHT][1];
	

	roger->base_setpoint[X] = v[path[where_in_path]].q[0];
	roger->base_setpoint[Y] = v[path[where_in_path]].q[1];
	roger->base_setpoint[2] = v[path[where_in_path]].q[2];
	roger->arm_setpoint[LEFT][0] = v[path[where_in_path]].q[3];
	roger->arm_setpoint[LEFT][1] = v[path[where_in_path]].q[4];
	roger->arm_setpoint[RIGHT][0] = v[path[where_in_path]].q[5];
	roger->arm_setpoint[RIGHT][1] = v[path[where_in_path]].q[6];
	
		
	distance = sqrt(pow(q[0]-v[path[where_in_path]].q[0],2)+pow(q[1]-v[path[where_in_path]].q[1],2));
	//distance = get_edge_distance(q,v[path[where_in_path]].q);
	//printf("Where in path : %d --> %lf\n",where_in_path,distance);
	//printf("Goal : %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",v[where_in_path].q[0],v[where_in_path].q[1],v[where_in_path].q[2],
	//	   v[where_in_path].q[3],v[where_in_path].q[4],v[where_in_path].q[5],v[where_in_path].q[6]);
	if (distance < 0.3)
{
		where_in_path--;
       scanf("%d",&tmp);
}
	/*
	if(where_in_path == -1)
	{
		roger->base_setpoint[X] = v[path[where_in_path]].q[0];
		roger->base_setpoint[Y] = v[path[where_in_path]].q[1];
		//roger->base_setpoint[2] = v[where_in_path].q[2];
		roger->arm_setpoint[LEFT][0] = v[path[where_in_path]].q[3];
		roger->arm_setpoint[LEFT][1] = v[path[where_in_path]].q[4];
		roger->arm_setpoint[RIGHT][0] = v[path[where_in_path]].q[5];
		roger->arm_setpoint[RIGHT][1] = v[path[where_in_path]].q[6];
		
	}*/
if(where_in_path == -1)
{
roger->base_setpoint[X] = goal[0];
		roger->base_setpoint[Y] = goal[1];
		//roger->base_setpoint[2] = goal[2];
		roger->arm_setpoint[LEFT][0] = goal[3];
		roger->arm_setpoint[LEFT][1] = goal[4];
		roger->arm_setpoint[RIGHT][0] = goal[5];
		roger->arm_setpoint[RIGHT][1] = goal[6];
}
	
	if(where_in_path < -1)
		got_plan = FALSE;
		
	/*
	double q[7], goal_near[7], q_near[7];
	double distance=1;
	int id1, id2;
	int i;
	
	id1 = nearest_neighbor_cartesian(goal, goal_near);
	q[0] = roger->base_position[0];
	q[1] = roger->base_position[1];
	q[2] = roger->base_position[2];
	q[3] = roger->arm_theta[LEFT][0];
	q[4] = roger->arm_theta[LEFT][1];
	q[5] = roger->arm_theta[RIGHT][0];
	q[6] = roger->arm_theta[RIGHT][1];
	
	id2 = nearest_neighbor_cartesian(q, q_near);
	printf("id1 : %d, id2 : %d\n",id1, id2);
	get_path(id1, id2);
	for(i =0 ; i<path_index; i++)
		printf("Path %d : %d \n",i,path[i]);
	
	for(i = 0; i<path_index; i++)
	{
		roger->base_setpoint[X] = v[i].q[0];
		roger->base_setpoint[Y] = v[i].q[1];
		roger->base_setpoint[2] = v[i].q[2];
		roger->arm_setpoint[LEFT][0] = v[i].q[3];
		roger->arm_setpoint[LEFT][1] = v[i].q[4];
		roger->arm_setpoint[RIGHT][0] = v[i].q[5];
		roger->arm_setpoint[RIGHT][1] = v[i].q[6];
		
		do
		{
			q[0] = roger->base_position[0];
			q[1] = roger->base_position[1];
			q[2] = roger->base_position[2];
			q[3] = roger->arm_theta[LEFT][0];
			q[4] = roger->arm_theta[LEFT][1];
			q[5] = roger->arm_theta[RIGHT][0];
			q[6] = roger->arm_theta[RIGHT][1];
			
			distance = get_edge_distance(q,v[where_in_path].q);
		}while(distance > 0.3);
	}*/
	
	

	/*
	 do
	 {
	 q[0] = roger->base_position[0];
	 q[1] = roger->base_position[1];
	 q[2] = roger->base_position[2];
	 q[3] = roger->arm_theta[LEFT][0];
	 q[4] = roger->arm_theta[LEFT][1];
	 q[5] = roger->arm_theta[RIGHT][0];
	 q[6] = roger->arm_theta[RIGHT][1];
	 
	 id2 = nearest_neighbor_cartesian(q, q_near);	
	 roger->base_setpoint[X] = q_near[0];
	 roger->base_setpoint[Y] = q_near[1];
	 roger->base_setpoint[2] = q_near[2];
	 roger->arm_setpoint[LEFT][0] = q_near[3];
	 roger->arm_setpoint[LEFT][1] = q_near[4];
	 roger->arm_setpoint[RIGHT][0] = q_near[5];
	 roger->arm_setpoint[RIGHT][1] = q_near[6];
	 printf("Goal id : %d, config id : %d , num_vertices : %d\n",id1, id2, num_vertices);
	 printf("Goal : %lf, %lf, %lf, %lf, %lf, %lf, %lf\n",goal[0],goal[1],goal[2],goal[3],goal[4],goal[5],goal[6]);
	 }while(id1 != id2);
	 */
}

void rrt_planning(roger, x_goal, y_goal)
Robot * roger;
double x_goal, y_goal;
{
	double q[7];
	//int draw_doorway();
	//build_rrt(roger);
	
	//draw_doorway(roger);
	printf("Reference : %lf, %lf \n",x_goal,y_goal);
	got_plan = FALSE;
	path_index = 0;
	set_goal(roger, x_goal, y_goal);
	build_rrt(roger);
	got_plan = TRUE;
	plan(roger);
	/*	q[0] = roger->base_position[0];
	 q[1] = roger->base_position[1];
	 q[2] = roger->base_position[2];
	 q[3] = roger->arm_theta[LEFT][0];
	 q[4] = roger->arm_theta[LEFT][1];
	 q[5] = roger->arm_theta[RIGHT][0];
	 q[6] = roger->arm_theta[RIGHT][1];
	 robot_in_collision(roger, q);*/
}
