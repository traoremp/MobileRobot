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
			//std::cout << "STATE: A" << std::endl;
			getOutofStart(cvs);
			break;

		case GAME_STATE_B:
			//std::cout << "STATE: B" << std::endl;
			cvs->path->goal_pos[X] = 100;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;
			
		case GAME_STATE_C:
			//std::cout << "STATE: C" << std::endl;
			//goal interm pour sortir facilement de la cage
			cvs->path->goal_pos[X] = 400;
			cvs->path->goal_pos[Y] = 100;
			follow_path(cvs);
			break;

		case GAME_STATE_D:
			//std::cout << "STATE: D" << std::endl;
			cvs->path->goal_pos[X] = 250;
			cvs->path->goal_pos[Y] = 1250;
			follow_path(cvs);
			break;

		case GAME_STATE_E:
			//std::cout << "STATE: E" << std::endl;
			cvs->path->goal_pos[X] = 600;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_F:
			//std::cout << "STATE: F" << std::endl;
			cvs->outputs->flag_release = 1; //lacher les cibles
			cvs->strat->main_state += 1;//next startegy state
			break;

		case GAME_STATE_G:
			//std::cout << "STATE: G" << std::endl;
			cvs->path->goal_pos[X] = 250;
			cvs->path->goal_pos[Y] = -1250;
			follow_path(cvs);
			break;

		case GAME_STATE_H:
			//std::cout << "STATE: H" << std::endl;
			cvs->path->goal_pos[X] = -400;
			cvs->path->goal_pos[Y] = -600;
			follow_path(cvs);
			break;

		case GAME_STATE_I:
			//std::cout << "STATE: I" << std::endl;
			goToBase(cvs);
			break;
		
		case GAME_STATE_J:
			//std::cout << "STATE: J" << std::endl;
			getOutofBase(cvs);
			break;

		case GAME_STATE_K:
			//std::cout << "STATE: K" << std::endl;
			cvs->path->goal_pos[X] = 700;
			cvs->path->goal_pos[Y] = -600;
			follow_path(cvs);
			break;

		case GAME_STATE_L:
			//std::cout << "STATE: L" << std::endl;
			cvs->path->goal_pos[X] = 700;
			cvs->path->goal_pos[Y] = 600;
			follow_path(cvs);
			break;

		case GAME_STATE_M:
			//std::cout << "STATE: M" << std::endl;
			goToBase(cvs);
			break;

		case GAME_STATE_N:
			//std::cout << "STATE: N" << std::endl;
			getOutofBase(cvs);
			break;

		case GAME_STATE_O:
			//std::cout << "STATE: O" << std::endl;
			cvs->path->goal_pos[X] = 600;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_P:
			//std::cout << "STATE: P" << std::endl;
			cvs->path->goal_pos[X] = 600;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			break;

		case GAME_STATE_Q:
			//std::cout << "STATE: Q" << std::endl;
			goToBase(cvs);
			break;

		case GAME_STATE_R:
			//std::cout << "STATE: R" << std::endl;
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

	//std::cout << angle << std::endl;

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
				cvs->path->goal_pos[X] = -700;
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
				cvs->path->goal_pos[X] = -700;
				cvs->path->goal_pos[Y] = 1200;
			}
			break;
	}

	follow_path(cvs);
}

void getOutofBase(CtrlStruct *cvs)
{
	cvs->outputs->flag_release = 1; //lacher les cibles

	if (cvs->rob_pos->theta <0 && cvs->rob_pos->theta > -M_PI + 0.05)
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
	}
}

void getOutofStart(CtrlStruct *cvs)
{
	switch (cvs->team_id)
	{
	case TEAM_A:
		if (cvs->rob_pos->y>0.4)
		{
			speed_regulation(cvs, 60.0, 60.0);
		}
		else
		{
			cvs->strat->main_state += 1;//next startegy state
		}
		break;
	case TEAM_B:
		if (cvs->rob_pos->y<-0.4)
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
