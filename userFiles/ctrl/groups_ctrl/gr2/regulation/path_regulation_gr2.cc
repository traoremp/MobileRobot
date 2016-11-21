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
	int rob_pos[COORDS];
	int goal_pos[COORDS] = {0,0};
	
	//attractive and repulsive forces
	float F_att[COORDS];
	float F_rep[COORDS];
	float F_tot[COORDS];
	int obst_pos[COORDS];
	
	int obstacle_dist = 0;
	float obstacle_theta = 0;
	int found = 0; 
	int d_min = 0;
	float alpha;
	
	cvs->path->map[goal_pos[I]][goal_pos[J]] = GOAL;
	rob_pos[I] = cvs->rob_pos->y*-1000-1500;
	rob_pos[J] = cvs->rob_pos->x*1000-1000;
	
	// --- Potential Field algorithm (start) --- //
	
	F_att[I] = -K_ATT*(rob_pos[I]-goal_pos[I]);
	F_att[J] = -K_ATT*(rob_pos[J]-goal_pos[J]);
	while (norm_dist(F_att[I],F_att[J]) > F_ATT_MAX)
	{
		F_att[I] = F_att[I]-sign(F_att[I]);
		F_att[J] = F_att[J]-sign(F_att[J]);
	}
	
	while (norm_dist(F_att[I],F_att[J]) < F_ATT_MIN)
	{
		F_att[I] = F_att[I]+sign(F_att[I]);
		F_att[J] = F_att[J]+sign(F_att[J]);
	}
	
	//get closest obstacle distance and angle
	while (found == 0 && d_min <= DIST_THRESHOLD) // tant qu'on a pas trouvé d'obstacle et que la zone de recherche est < threshold
	{
		for (alpha = -M_PI/2; alpha <= M_PI/2; alpha = alpha + M_PI/4) //check 5 direction around robot
			if(rob_pos[I]+(int)((d_min+ROBOT_SIZE)*cos(alpha)) < MAP_LENGTH && rob_pos[J]+(int)((d_min+ROBOT_SIZE)*sin(alpha)) < MAP_WIDTH && rob_pos[I]+(int)((d_min+ROBOT_SIZE)*cos(alpha)) > 0 && rob_pos[J]+(int)((d_min+ROBOT_SIZE)*sin(alpha)) > 0)
				if (cvs->path->map[rob_pos[I]+(int)((d_min+ROBOT_SIZE)*cos(alpha))][rob_pos[J]+(int)((d_min+ROBOT_SIZE)*sin(alpha))] == 1) // if obstacle
				{
					found = 1;
					obstacle_theta = alpha;
					obstacle_dist = d_min;                
				}
		d_min = d_min +1;
	}
	
	if (obstacle_dist==0) // =collision
		obstacle_dist = 1; // avoid infinite repulsive force
	
	if (obstacle_dist >= DIST_THRESHOLD)
	{
		F_rep[I] = 0;
		F_rep[J] = 0;
	}
	else
	{
		obst_pos[I] = rob_pos[I]+round(obstacle_dist*cos(obstacle_theta));
		obst_pos[J] = rob_pos[J]+round(obstacle_dist*sin(obstacle_theta));
		F_rep[I] = -K_REP*(1/obstacle_dist)*cos(obstacle_theta);
		F_rep[J] = -K_REP*(1/obstacle_dist)*sin(obstacle_theta);
	}
	while (norm_dist(F_rep[I], F_rep[J])>F_REP_MAX)
	{
		F_rep[I] = F_rep[I]-sign(F_rep[I]);
		F_rep[J] = F_rep[J]-sign(F_rep[J]);
	}

    F_tot[I] = F_att[I] + F_rep[I];
	F_tot[J] = F_att[J] + F_rep[J];
	
	// --- Potential Field algorithm (end) --- //
	
	// --- Force to motors command transformation (sart) --- //
	
	// --- Force to motors command transformation (end) --- //
}

//set the command to motors by transforming the vector F_tot
void ForceToCommand(float F[], CtrlStruct *cvs)
{
	float w[COORDS] = {0,0};
	
    if (F[I] || F[J])
	{
        vecteur_pos_rob = [cos(rob_theta) sin(rob_theta)];

        alpha = acos((F*vecteur_pos_rob')/(norm(F)*norm(vecteur_pos_rob)));
        %limite angle -pi/pi
        while (alpha>=pi) alpha = alpha-2*pi; end
        while (alpha>=pi) alpha = alpha-2*pi; end
        
        //sign of the angle 
        alpha = sign(det([vecteur_pos_rob' F']))*alpha;

        ampl = norm_dist(F[I], F[J]);

        wl = ampl - rot_speed*alpha/pi*ampl;
        wr = ampl + rot_speed*alpha/pi*ampl;
    }

}
NAMESPACE_CLOSE();
