#include "path_regulation_gr2.h"
#include "useful_gr2.h"
#include "speed_regulation_gr2.h"
#include "init_pos_gr2.h"
#include "path_planning_gr2.h"
#include "strategy_gr2.h"
#include <iostream>
#include "opp_pos_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{		
	//attractive and repulsive forces
	double F_att[COORDS];
	double F_rep[COORDS];
	double F_opp[COORDS];
	double F_tot[COORDS];
	int obst_pos[COORDS];

	double turn = 0.0;

	int rob_posx;
	int rob_posy;
	int goal_posx;
	int goal_posy;
	int opp_posx;
	int opp_posy;
	int team_base_x;
	int team_base_y;

	double obstacle_theta = 0;
	double opp_theta = 0;
	bool found = false; 
	double alpha;
	double alpha_rel;

	switch (cvs->team_id)
	{
	case TEAM_A:
		team_base_x = -500;
		team_base_y = -800;
		break;

	case TEAM_B:
		team_base_x = -500;
		team_base_y = 800;
		break;

	default:
		break;
	}

	cvs->outputs->flag_release = 0;

	rob_posx = cvs->rob_pos->x * 1000;
	rob_posy = cvs->rob_pos->y * 1000;
	
	goal_posx = cvs->path->goal_pos[X];
	goal_posy = cvs->path->goal_pos[Y];

	opp_posx = cvs->opp_pos->x[0] * 1000;
	opp_posy = cvs->opp_pos->y[0] * 1000;

	// --- Potential Field algorithm (start) --- //
	
	F_att[X] = -K_ATT*(rob_posx - goal_posx);
	F_att[Y] = -K_ATT*(rob_posy - goal_posy);

	//limit F_att
	//limitNorm(F_att, F_ATT_MAX, F_ATT_MIN);

	//std::cout << "F_att" << F_att[X] << ",";
	//std::cout << F_att[Y] << std::endl;
	
	//get closest obstacle distance and angle
	int obstacle_dist = DIST_THRESHOLD;
	int opp_dist;
	int d_computed;
	int ob_number = 0;
	int cas_nb = 0;
	int ob_cas_nb = 0;


	//UPDATE OPPENENT POSITION
	//cvs->path->map[22][OB_X] = cvs->opp_pos->x[0] * 1000;
	//cvs->path->map[22][OB_Y] = cvs->opp_pos->y[0] * 1000;

	if ((goal_posx == 100 && goal_posy == 0) || (rob_posx>-100 && rob_posx<100 && rob_posx>-300 && rob_posy<300))
	{
		cvs->path->cage_open = 1;//cage open
	}
	else
	{
		cvs->path->cage_open = 0;//cage closed
	}

	for (int i = 0; i < NB_RECT-cvs->path->cage_open; i++)
	{
		if (rob_posx<cvs->path->map[i][OB_X]- cvs->path->map[i][OB_L]/2 && rob_posy>cvs->path->map[i][OB_Y]+ cvs->path->map[i][OB_H]/2)
		{
			d_computed = sqrt( pow(cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 - rob_posx,2) + pow(rob_posy-cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2,2) ) - ROBOT_SIZE;
			cas_nb = 1;
		}
		else if (rob_posx<cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 && rob_posy>cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2 && rob_posy<cvs->path->map[i][OB_Y] + cvs->path->map[i][OB_H] / 2)
		{
			d_computed = abs(cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 - rob_posx - ROBOT_SIZE);
			cas_nb = 2;
		}
		else if (rob_posx<cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 && rob_posy<cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2)
		{
			d_computed = sqrt(pow(cvs->path->map[i][OB_X]- cvs->path->map[i][OB_L] / 2-rob_posx, 2) + pow(cvs->path->map[i][OB_Y]- cvs->path->map[i][OB_H] / 2-rob_posy, 2)) - ROBOT_SIZE;
			cas_nb = 3;
		}
		else if (rob_posx<cvs->path->map[i][OB_X] + cvs->path->map[i][OB_L] / 2 && rob_posx>cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 && rob_posy<cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2)
		{
			d_computed = abs(cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2 - rob_posy - ROBOT_SIZE);
			cas_nb = 4;
		}
		else if (rob_posx>cvs->path->map[i][OB_X] + cvs->path->map[i][OB_L] / 2 && rob_posy<cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2)
		{
			d_computed = sqrt(pow(rob_posx- cvs->path->map[i][OB_X]- cvs->path->map[i][OB_L] / 2, 2) + pow(cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2 - rob_posy, 2)) - ROBOT_SIZE;
			cas_nb = 5;
		}
		else if (rob_posx>cvs->path->map[i][OB_X] + cvs->path->map[i][OB_L] / 2 && rob_posy<cvs->path->map[i][OB_Y] + cvs->path->map[i][OB_H] / 2 && rob_posy>cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2)
		{
			d_computed = abs(rob_posx - cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 - ROBOT_SIZE);
			cas_nb = 6;
		}
		else if (rob_posx>cvs->path->map[i][OB_X] + cvs->path->map[i][OB_L] / 2 && rob_posy>cvs->path->map[i][OB_Y] + cvs->path->map[i][OB_H] / 2)
		{
			d_computed = sqrt(pow(rob_posx- cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2, 2) + pow(rob_posy- cvs->path->map[i][OB_Y]- cvs->path->map[i][OB_H] / 2, 2)) - ROBOT_SIZE;
			cas_nb = 7;
		}
		else if (rob_posx<cvs->path->map[i][OB_X] + cvs->path->map[i][OB_L] / 2 && rob_posx>cvs->path->map[i][OB_X] - cvs->path->map[i][OB_L] / 2 && rob_posy>cvs->path->map[i][OB_Y] + cvs->path->map[i][OB_H] / 2)
		{
			d_computed = abs(rob_posy - cvs->path->map[i][OB_Y] - cvs->path->map[i][OB_H] / 2 - ROBOT_SIZE);
			cas_nb = 8;
		}

		if (d_computed < obstacle_dist) {
			obstacle_dist = d_computed;
			ob_number = i;
			ob_cas_nb = cas_nb;
		}
	}
	switch (ob_cas_nb)
	{
		case 1:
			obstacle_theta = atan2(rob_posy- cvs->path->map[ob_number][OB_Y] - cvs->path->map[ob_number][OB_H] / 2, cvs->path->map[ob_number][OB_X] - cvs->path->map[ob_number][OB_L] / 2 - rob_posx);
			break;
		case 2:
			obstacle_theta = 0;
			break;
		case 3:
			obstacle_theta = atan2(cvs->path->map[ob_number][OB_Y] - cvs->path->map[ob_number][OB_H] / 2 - rob_posy, cvs->path->map[ob_number][OB_X] - cvs->path->map[ob_number][OB_L] / 2 - rob_posx);
			break;
		case 4:
			obstacle_theta = M_PI / 2;
			break;
		case 5:
			obstacle_theta = atan2(cvs->path->map[ob_number][OB_Y] - cvs->path->map[ob_number][OB_H] / 2 - rob_posy, cvs->path->map[ob_number][OB_X] + cvs->path->map[ob_number][OB_L] / 2 -rob_posx);
			break;
		case 6:
			obstacle_theta = -M_PI;
			break;
		case 7:
			obstacle_theta = atan2(cvs->path->map[ob_number][OB_Y] + cvs->path->map[ob_number][OB_H] / 2 - rob_posy, cvs->path->map[ob_number][OB_X] + cvs->path->map[ob_number][OB_L] / 2 - rob_posx);
			break;
		case 8:
			obstacle_theta = -M_PI / 2;
			break;
		default:
			break;
	}
	
	if (obstacle_dist >= DIST_THRESHOLD || obstacle_dist == 0)
	{
		F_rep[X] = 0;
		F_rep[Y] = 0;
	}
	else
	{
		obst_pos[X] = rob_posx+round(obstacle_dist*cos(obstacle_theta));
		obst_pos[Y] = rob_posy+round(obstacle_dist*sin(obstacle_theta));
		//F_rep[I] = -K_REP*(1/obstacle_dist)*sin(obstacle_theta);
		//F_rep[J] = -K_REP*(1/obstacle_dist)*cos(obstacle_theta)
		F_rep[X] = K_REP*(1.0 / obstacle_dist - 1.0 / DIST_THRESHOLD)*(rob_posx - obst_pos[X]) /pow(obstacle_dist,2);
		F_rep[Y] = K_REP*(1.0 / obstacle_dist - 1.0 / DIST_THRESHOLD)*(rob_posy - obst_pos[Y]) / pow(obstacle_dist,2);

	}


	/*std::cout << "F_att: " << F_att[X] << ",";
	std::cout << F_att[Y] << std::endl;
	std::cout << "F_rep " << F_rep[X] << ",";
	std::cout << F_rep[Y] << std::endl;
	std::cout << "obst_dist: " << obstacle_dist << ",";
	std::cout << "obst_cas_nb: " << ob_cas_nb << ",";
	std::cout << "obst_theta: " << obstacle_theta << std::endl;*/

	F_tot[X] = F_att[X] + F_rep[X];
	F_tot[Y] = F_att[Y] + F_rep[Y];


	//Goal reached
	if (cvs->path->wait)
	{
		F_tot[X] = 0;
		F_tot[Y] = 0;

		turn = 10;

		if (!cvs->inputs->target_detected)
		{
			cvs->inputs->nb_targets += 1;
			cvs->path->wait = 0;
			cvs->strat->main_state += 1;//next startegy state
		}
	}
	else if (norm_dist(rob_posx - cvs->path->goal_pos[X], rob_posy - cvs->path->goal_pos[Y]) < 100 && cvs->inputs->target_detected && (rob_posx>team_base_x || rob_posy>team_base_y))
	{
		cvs->path->wait = 1;
		F_tot[X] = 0;
		F_tot[Y] = 0;
		
	}
	else if (norm_dist(rob_posx - cvs->path->goal_pos[X], rob_posy - cvs->path->goal_pos[Y]) < 50)
	{
		cvs->strat->main_state += 1;//next startegy state
	}

	// Opponent force computation
	opp_dist = norm_dist(rob_posx - opp_posx, rob_posy - opp_posy) - ROBOT_SIZE;
	opp_theta = atan2((opp_posy-rob_posy) , (opp_posx - rob_posx));
	//std::cout << opp_dist << std::endl;
	//std::cout << opp_theta << std::endl;
	
	if (opp_dist < 3*DIST_THRESHOLD && opp_dist!=0)
	{
		F_opp[X] = K_OPP*(1.0 / opp_dist - 1.0 / (3*DIST_THRESHOLD))*(rob_posx - (rob_posx + round(opp_dist*cos(opp_theta)))) / pow(opp_dist, 2);
		F_opp[Y] = K_OPP*(1.0 / opp_dist - 1.0 / (3*DIST_THRESHOLD))*(rob_posy - (rob_posy + round(opp_dist*sin(opp_theta)))) / pow(opp_dist, 2);
		//std::cout << F_opp[X] << std::endl;
		//std::cout << F_opp[Y] << std::endl;
		//F_tot[X] = 0;
		//F_tot[Y] = 0;
	}
	else
	{
		F_opp[X] = 0;
		F_opp[Y] = 0;
	}

	F_tot[X] = F_tot[X] + F_opp[X];
	F_tot[Y] = F_tot[Y] + F_opp[Y];

	// --- Potential Field algorithm (end) --- //
	
	// --- Force to motors command transformation (sart) --- //

	ForceToCommand(F_tot, cvs, turn);
	
	// --- Force to motors command transformation (end) --- //
}

//set the command to motors by transforming the vector F_tot
void ForceToCommand(double F[], CtrlStruct *cvs, double turn)
{
	double wl =0;
	double wr =0;

	double vecteur_pos_rob[COORDS];
	double alpha;
	double ampl;
	
    if (F[X] || F[Y])
	{
        vecteur_pos_rob[X] = cos(cvs->rob_pos->theta);
        vecteur_pos_rob[Y] = sin(cvs->rob_pos->theta);

		alpha = atan2(F[Y], F[X]) - cvs->rob_pos->theta;
		alpha = limit_angle(alpha);

		ampl = norm_dist(F[X], F[Y]);
		ampl = ampl > 80 ? 80 : ampl ;
		ampl = ampl < 10 ? 10 : ampl;

        wl = 20 - 2*ROT_SPEED*alpha/M_PI*20;
        wr = 20 + 2*ROT_SPEED*alpha/M_PI*20;
    }
    else if (turn)
   	{
   		wl = turn;
   		wr = -turn;
   	}
	else
	{
		wl = 0.0;
		wr = 0.0;
	}

   	speed_regulation(cvs, wr, wl);

}

NAMESPACE_CLOSE();
