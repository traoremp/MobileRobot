#include "path_regulation_gr2.h"
#include "useful_gr2.h"
#include "speed_regulation_gr2.h"
#include "init_pos_gr2.h"
#include "path_planning_gr2.h"
#include "strategy_gr2.h"
#include <iostream>

NAMESPACE_INIT(ctrlGr2);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{	
	int rob_pos[COORDS];
		
	//attractive and repulsive forces
	float F_att[COORDS] = { 0,0 };
	float F_rep[COORDS] = { 0,0 };
	float F_tot[COORDS] = { 0,0 };
	int obst_pos[COORDS];
	
	int obstacle_dist = 0;
	float obstacle_theta = 0;
	int found = 0; 
	int d_min = 0;
	float alpha;
	float alpha_rel;
	float t;

	t = cvs->inputs->t;
	
	cvs->path->map[cvs->path->goal_pos[I]][cvs->path->goal_pos[J]] = GOAL;
	rob_pos[I] = (int)((1500 - cvs->rob_pos->y*1000)/CELL_SIZE);
	rob_pos[J] = (int)((1000 + cvs->rob_pos->x*1000)/CELL_SIZE);
	
	// --- Potential Field algorithm (start) --- //
	
	F_att[I] = -K_ATT*(rob_pos[I]- cvs->path->goal_pos[I]);
	F_att[J] = -K_ATT*(rob_pos[J]- cvs->path->goal_pos[J]);

	//limit F_att
	limitNorm(F_att, F_ATT_MAX, F_ATT_MIN);


	//std::cout << "F_att" << F_att[I] << ",";
	//std::cout << F_att[J] << std::endl;
	
	//get closest obstacle distance and angle
	while (found == 0 && d_min <= DIST_THRESHOLD) // tant qu'on a pas trouvé d'obstacle et que la zone de recherche est < threshold
	{
		for (alpha = -M_PI / 2; alpha <= M_PI / 2; alpha = alpha + M_PI / 8) //check 10 direction around robot
		{
			alpha_rel = alpha + cvs->rob_pos->theta;
			alpha_rel = limit_angle(alpha_rel);
			if (rob_pos[I] + (int)((d_min + ROBOT_SIZE)*cos(alpha_rel)) < MAP_LENGTH && rob_pos[J] + (int)((d_min + ROBOT_SIZE)*sin(alpha_rel)) < MAP_WIDTH && rob_pos[I] + (int)((d_min + ROBOT_SIZE)*cos(alpha_rel)) > 0 && rob_pos[J] + (int)((d_min + ROBOT_SIZE)*sin(alpha_rel)) > 0)
				if (cvs->path->map[rob_pos[I] - (int)((d_min + ROBOT_SIZE)*sin(alpha_rel))][rob_pos[J] + (int)((d_min + ROBOT_SIZE)*cos(alpha_rel))] == 1) // if obstacle
				{
					found = 1;
					obstacle_theta = alpha_rel;
					obstacle_dist = d_min;
				}
		}			
		d_min = d_min +1;
	}

	
	//if (obstacle_dist==0) // =collision
		//obstacle_dist = 1; // avoid infinite repulsive force
	
	if (obstacle_dist >= DIST_THRESHOLD || obstacle_dist == 0)
	{
		F_rep[I] = 0;
		F_rep[J] = 0;
	}
	else
	{
		obst_pos[I] = rob_pos[I]-round(obstacle_dist*sin(obstacle_theta));
		obst_pos[J] = rob_pos[J]+round(obstacle_dist*cos(obstacle_theta));
		//F_rep[I] = -K_REP*(1/obstacle_dist)*sin(obstacle_theta);
		//F_rep[J] = -K_REP*(1/obstacle_dist)*cos(obstacle_theta)
		F_rep[I] = K_REP*(1.0 / obstacle_dist - 1.0 / DIST_THRESHOLD)*(rob_pos[I] - obst_pos[I]) /pow(obstacle_dist,3);
		F_rep[J] = K_REP*(1.0 / obstacle_dist - 1.0 / DIST_THRESHOLD)*(rob_pos[J] - obst_pos[J]) / pow(obstacle_dist,3);

	}

	//limitNorm(F_rep, F_REP_MAX, 0);

	/*std::cout << "F_att: " << F_att[I] << ",";
	std::cout << F_att[J] << std::endl;
	std::cout << "F_rep " << F_rep[I] << ",";
	std::cout << F_rep[J] << std::endl;
	std::cout << "obst_dist: " << obstacle_dist << ",";
	std::cout << "obst_theta: " << obstacle_theta << std::endl;*/

	F_tot[I] = F_att[I] +F_rep[I];
	F_tot[J] = F_att[J] +F_rep[J];

	if (norm_dist(rob_pos[I] - cvs->path->goal_pos[I], rob_pos[J] - cvs->path->goal_pos[J]) < 1 && !cvs->path->wait) //Goal reached
	{
		cvs->path->last_t = t;
		cvs->path->wait = 1;
		F_tot[I] = 0;
		F_tot[J] = 0;
		
	}
	else if (norm_dist(rob_pos[I] - cvs->path->goal_pos[I], rob_pos[J] - cvs->path->goal_pos[J]) < 1 && cvs->path->wait)
	{
		F_tot[I] = 0;
		F_tot[J] = 0;

		if (t - cvs->path->last_t > 2)
		{
			cvs->path->wait = 0;
			cvs->strat->main_state += 1;//next startegy state
		}
	}

	
	// --- Potential Field algorithm (end) --- //
	
	// --- Force to motors command transformation (sart) --- //

	ForceToCommand(F_tot, cvs);
	
	// --- Force to motors command transformation (end) --- //
}

//set the command to motors by transforming the vector F_tot
void ForceToCommand(float F[], CtrlStruct *cvs)
{
	float wl =0;
	float wr =0;

	float vecteur_pos_rob[COORDS];
	float alpha;
	float ampl;
	
    if (F[I] || F[J])
	{
        vecteur_pos_rob[I] = cos(cvs->rob_pos->theta + M_PI/2);
        vecteur_pos_rob[J] = sin(cvs->rob_pos->theta + M_PI/2);

		if (abs((F[I] * vecteur_pos_rob[I] + F[J] * vecteur_pos_rob[J]) / (norm_dist(F[I], F[J])*norm_dist(vecteur_pos_rob[I], vecteur_pos_rob[J]))) > 1)
		{
			alpha = 0;
		}
		else
		{
			alpha = acos((F[I] * vecteur_pos_rob[I] + F[J] * vecteur_pos_rob[J]) / (norm_dist(F[I], F[J])*norm_dist(vecteur_pos_rob[I], vecteur_pos_rob[J])));
		}
	
        //limite angle -pi/pi
        alpha = limit_angle(alpha);
		       
        //sign of the angle 
		alpha = sign(Det2X2Matrix(vecteur_pos_rob, F))*alpha;

		ampl = norm_dist(F[I], F[J]);
		ampl = ampl > 50 ? 50 : ampl ;

        wl = ampl - ROT_SPEED*alpha/M_PI*ampl;
        wr = ampl + ROT_SPEED*alpha/M_PI*ampl;
    }
    else
   	{
   		wl = 0;
   		wr = 0;
   	}

   	speed_regulation(cvs, wr, wl);

}

void limitNorm(float F[], int maxNorm, int minNorm)
{
	float ref_angle = M_PI / 2;

	if (F[J])
	{
		ref_angle = atan(F[I] / F[J]);
		if (F[J] < 0 && F[I]>0)
			ref_angle += M_PI;
		else if (F[J]<0 && F[I]<0)
			ref_angle += -M_PI;
	}
	else
	{
		if (F[I]>0)
			ref_angle = M_PI / 2;
		if (F[J]<0)
			ref_angle = -M_PI / 2;
	}


	if (norm_dist(F[I], F[J]) > maxNorm)
	{
		F[I] = maxNorm * sin(ref_angle);
		F[J] = maxNorm * cos(ref_angle);
	}

	if (norm_dist(F[I], F[J]) < minNorm && norm_dist(F[I], F[J]) != 0)
	{
		F[I] = minNorm * sin(ref_angle);
		F[J] = minNorm * cos(ref_angle);
	}
}

NAMESPACE_CLOSE();
