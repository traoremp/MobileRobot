#include "path_regulation_gr2.h"
#include "useful_gr2.h"
#include "speed_regulation_gr2.h"
#include "init_pos_gr2.h"
#include "path_planning_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{
	//variables used for potential field algorithm
	//float dt = 0.1;
	
	int distanceThreshold = 20; //threshold for repulsive field
	float k_att = 0.1;
	int k_rep = 600;
	int F_att_max = 30;
	int F_att_min = 12;
	int F_rep_max = 50;
	int rob_pos[COORDS];
	int goal_pos[COORDS] = {0,0};
	
	float rot_speed = 1.8; //constante used for forcetocommand to specify the rotation speed 

	int v_max = 30; //vitesse angulaire max des roues
	
	//attractive and repulsive forces
	float F_att[COORDS];
	float F_rep[COORDS];
	float F_tot[COORDS];
	
	int obstacle_dist = 0;
	float obstacle_theta = 0;
	int found = 0; 
	int d_min = 0;
	float alpha;
	
	cvs->path->map[goal_pos[X]][goal_pos[Y]] = GOAL;
	rob_pos[I] = cvs->rob_pos->y*-1000-1500;
	rob_pos[J] = cvs->rob_pos->x*1000-1000;
	
	/*F_att[X] = -k_att*(rob_pos[X]-goal_pos[X]);
	F_att[Y] = -k_att*(rob_pos[Y]-goal_pos[Y]);
	while (sqrt(F_att[X]*F_att[X] + F_att[Y]*F_att[Y])>F_att_max) //norme
	{
		if(F_att[X] > 0)
			F_att[X] = F_att[X]-1;
		else if (F_att[X] < 0)
			F_att[X] = F_att[X]+1;
			
		if(F_att[Y] > 0)
			F_att[Y] = F_att[Y]-1;
		else if(F_att[Y] < 0)
			F_att[Y] = F_att[Y]+1;
	}
	
	while (sqrt(F_att[X]*F_att[X] + F_att[Y]*F_att[Y])<F_att_min) //norme
	{
		if(F_att[X] > 0)
			F_att[X] = F_att[X]+1;
		else if (F_att[X] < 0)
			F_att[X] = F_att[X]-1;
			
		if(F_att[Y] > 0)
			F_att[Y] = F_att[Y]+1;
		else if(F_att[Y] < 0)
			F_att[Y] = F_att[Y]-1;
	}
	
	//get closest obstacle distance and angle
	while (found == 0 && d_min <= distanceThreshold) // tant qu'on a pas trouvé d'obstacle et que la zone de recherche est < threshold
	{
		for (alpha = -pi/2; alpha <= pi/2; alpha = alpha + pi/4:) //check 5 direction around robot
			if (path->map[rob_pos[X]+int16(d_min*cos(alpha))][rob_pos[Y]+int16(d_min*sin(alpha))] == 1) // if obstacle
			{
				found = 1;
				obstacle_theta = alpha;
				obstacle_dist = d_min;                
			}
		d_min = d_min +1;
	}
	
	if (obstacle_dist==0) // =collision
		obstacle_dist = 1; // avoid infinite repulsive force
	
	if (obstacle_dist >= distanceThreshold)
	{
		F_rep[X] = 0;
		F_rep[Y] = 0;
	}
	else
	{
		obst_pos[X] = rob_pos[X]+round(obstacle_dist*cos(obstacle_theta));
		obst_pos[Y] = rob_pos[Y]+round(obstacle_dist*sin(obstacle_theta));
		F_rep[X] = -k_rep*(1/obstacle_dist)*cos(obstacle_theta);
		F_rep[Y] = -k_rep*(1/obstacle_dist)*sin(obstacle_theta);
	}
	while (sqrt(F_rep[X]*F_rep[X] + F_rep[Y]*F_rep[Y])>F_rep_max)
	{
		if(F_rep[X] > 0)
			F_rep[X] = F_rep[X]-1;
		else if (F_rep[X] < 0)
			F_rep[X] = F_rep[X]+1;
			
		if(F_rep[Y] > 0)
			F_rep[Y] = F_rep[Y]-1;
		else if(F_rep[Y] < 0)
			F_rep[Y] = F_rep[Y]+1;
	}

    F_tot[X] = F_att[X] + F_rep[X];
	F_tot[Y] = F_att[Y] + F_rep[Y];*/

}

NAMESPACE_CLOSE();
