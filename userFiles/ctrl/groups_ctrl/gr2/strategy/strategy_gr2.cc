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
			cvs->path->goal_pos[X] = 100;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			//speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_B:
			goToBase(cvs);
			break;

		case GAME_STATE_C:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_D:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_E:
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

NAMESPACE_CLOSE();
