#include "strategy_gr2.h"
#include "path_planning_gr2.h"
#include "speed_regulation_gr2.h"
#include "path_regulation_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "odometry_gr2.h"
#include <math.h>
#include <iostream>

NAMESPACE_INIT(ctrlGr2);

/*! \brief intitialize the strategy structure
 * 
 * \return strategy structure initialized
 */
Strategy* init_strategy()
{
	Strategy *strat;

	strat = (Strategy*) malloc(sizeof(Strategy));

	return strat;
}

/*! \brief release the strategy structure (memory released)
 * 
 * \param[out] strat strategy structure to release
 */
void free_strategy(Strategy *strat)
{
	free(strat);
}

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */
void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	CtrlIn *inputs;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;


	switch (strat->main_state)
	{
		case GAME_STATE_A:
			getOutofStart(cvs);
			break;

		case GAME_STATE_B:
			cvs->path->goal_pos[X] = 100;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;
			
		case GAME_STATE_C:
			cvs->path->goal_pos[X] = 400;
			cvs->path->goal_pos[Y] = 100;
			follow_path(cvs);
			break;

		case GAME_STATE_D:
			cvs->path->goal_pos[X] = 250;
			cvs->path->goal_pos[Y] = 1250;
			follow_path(cvs);
			break;

		case GAME_STATE_E:
			cvs->path->goal_pos[X] = 600;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_F:
			cvs->outputs->flag_release = 1; //lacher les cibles
			cvs->strat->main_state += 1;//next startegy state
			break;

		case GAME_STATE_G:
			cvs->path->goal_pos[X] = 250;
			cvs->path->goal_pos[Y] = -1250;
			follow_path(cvs);
			break;

		case GAME_STATE_H:
			cvs->path->goal_pos[X] = -400;
			cvs->path->goal_pos[Y] = -600;
			follow_path(cvs);
			break;

		case GAME_STATE_I:
			goToBase(cvs);
			break;
		
		case GAME_STATE_J:
			getOutofBase(cvs);
			break;

		case GAME_STATE_K:
			cvs->path->goal_pos[X] = 700;
			cvs->path->goal_pos[Y] = -600;
			follow_path(cvs);
			break;

		case GAME_STATE_L:
			cvs->path->goal_pos[X] = 700;
			cvs->path->goal_pos[Y] = 600;
			follow_path(cvs);
			break;

		case GAME_STATE_M:
			goToBase(cvs);
			break;

		case GAME_STATE_N:
			getOutofBase(cvs);
			break;

		case GAME_STATE_O:
			cvs->path->goal_pos[X] = 600;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_P:
			cvs->path->goal_pos[X] = 600;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_Q:
			goToBase(cvs);
			break;

		case GAME_STATE_R:
			getOutofBase(cvs);
			break;

		case GAME_STATE_S:
			cvs->path->goal_pos[X] = -800;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_T:
			cvs->path->goal_pos[X] = -400;
			cvs->path->goal_pos[Y] = 600;
			follow_path(cvs);
			break;

		case GAME_STATE_U:
			goToBase(cvs);
			break;

		case GAME_STATE_V:
			cvs->outputs->flag_release = 1; //lacher les cibles
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

void goToBase(CtrlStruct *cvs)
{
	double distance = sqrt(pow(cvs->rob_pos->x + 0.7, 2) + pow(cvs->rob_pos->y + 1.2, 2));
	double angle = atan2(cvs->rob_pos->y + 1.2, cvs->rob_pos->x + 0.7);

	switch (cvs->team_id)
	{
		case TEAM_A:
			if (distance >= 1.204 && angle<1.16  && angle>0.72) //goal intermediaire
			{
				cvs->path->goal_pos[X] = 500;
				cvs->path->goal_pos[Y] = -400;
			}
			else
			{
				cvs->path->goal_pos[X] = -750;
				cvs->path->goal_pos[Y] = -1200;
			}
			break;
		case TEAM_B:
			if (distance >= 1.204 && angle<-0.72  && angle>-1.16) //goal intermediaire
			{
				cvs->path->goal_pos[X] = 500;
				cvs->path->goal_pos[Y] = 400;
			}
			else
			{
				cvs->path->goal_pos[X] = -750;
				cvs->path->goal_pos[Y] = 1200;
			}
			break;
	}

	follow_path(cvs);
}

void getOutofBase(CtrlStruct *cvs)
{
	cvs->outputs->flag_release = 1; //lacher les cibles

	double t = cvs->inputs->t;

	switch (cvs->path->BASE_STATE)
	{
	case BASE_STATE_1:
		speed_regulation(cvs, -15.0, 15.0);
		if (cvs->rob_pos->theta <M_PI / 2 - 0.005 && cvs->rob_pos->theta > M_PI / 2 - 0.02)
		{
			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_2:
		speed_regulation(cvs, -30.0, -30.0);
		if (cvs->inputs->u_switch[0] && cvs->inputs->u_switch[1])
		{
			cvs->path->last_t = t;
			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_3:
		speed_regulation(cvs, -30.0, -30.0);
		if (t - cvs->path->last_t > 0.2)
		{
			if (cvs->team_id == TEAM_A)
			{
				cvs->rob_pos->y = -1.5 + 0.06;
				cvs->rob_pos->theta = M_PI / 2;
			}
			else if (cvs->team_id == TEAM_B)
			{
				cvs->rob_pos->y = 0.85 + 0.06;
				cvs->rob_pos->theta = M_PI / 2;
			}
			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_4:
		speed_regulation(cvs, 30.0, 30.0);
		if ((cvs->rob_pos->y > -1.3 && cvs->team_id == TEAM_A) || (cvs->rob_pos->y > 1.2 && cvs->team_id == TEAM_B))
		{
			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_5:
		speed_regulation(cvs, -15.0, 15.0);
		if (cvs->rob_pos->theta < 0 && cvs->rob_pos->theta > - 0.02)
		{
			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_6:
		speed_regulation(cvs, -30.0, -30.0);
		if (cvs->inputs->u_switch[0] && cvs->inputs->u_switch[1])
		{
			cvs->path->last_t = t;
			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_7:
		speed_regulation(cvs, -30.0, -30.0);
		if (t - cvs->path->last_t > 0.2)
		{
			cvs->rob_pos->x = -1.0 + 0.06;
			cvs->rob_pos->theta = 0;

			cvs->path->BASE_STATE += 1;
		}
		break;
	case BASE_STATE_8:
		speed_regulation(cvs, 30.0, 30.0);
		if (cvs->rob_pos->x > -0.4)
		{
			cvs->path->BASE_STATE = 0;
			cvs->outputs->flag_release = 0;
			cvs->strat->main_state += 1;//next startegy state
		}
		break;
	default:
		break;
	}

	
	/*if (cvs->rob_pos->theta <0 && cvs->rob_pos->theta > -M_PI + 0.05)
	{
		speed_regulation(cvs, -30.0, 30.0);
	}
	else if (cvs->rob_pos->theta > 0 && cvs->rob_pos->theta < M_PI - 0.05)
	{
		speed_regulation(cvs, 30.0, -30.0);
	}
	else if (cvs->rob_pos->x < -0.2)
	{
		speed_regulation(cvs, -30.0, -30.0);
	}
	else
	{
		cvs->outputs->flag_release = 0;
		cvs->strat->main_state += 1;//next startegy state
	}*/
}

void getOutofStart(CtrlStruct *cvs)
{
	switch (cvs->team_id)
	{
	case TEAM_A:
		if (cvs->rob_pos->y>0.3)
		{
			speed_regulation(cvs, 60.0, 60.0);
		}
		else
		{
			cvs->strat->main_state += 1;//next startegy state
		}
		break;
	case TEAM_B:
		if (cvs->rob_pos->y<-0.3)
		{
			speed_regulation(cvs, 60.0, 60.0);
		}
		else
		{
			cvs->strat->main_state += 1;//next startegy state
		}
		break;
	}
}

NAMESPACE_CLOSE();
