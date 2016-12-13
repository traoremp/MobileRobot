#include "strategy_gr2.h"
#include "path_planning_gr2.h"
#include "speed_regulation_gr2.h"
#include "path_regulation_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "odometry_gr2.h"
#include <math.h>

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
			cvs->path->goal_pos[X] = -400;
			cvs->path->goal_pos[Y] = 600;
			follow_path(cvs);
			//speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_B:
			cvs->path->goal_pos[X] = -800;
			cvs->path->goal_pos[Y] = 0;
			follow_path(cvs);
			//speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_C:
			cvs->path->goal_pos[X] = -750;
			cvs->path->goal_pos[Y] = -1200;
			follow_path(cvs);
			//speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_D:
			cvs->path->goal_pos[X] = -750;
			cvs->path->goal_pos[Y] = -1200;
			follow_path(cvs);
			//speed_regulation(cvs, 0.0, 0.0);
			break;

		case GAME_STATE_E:
			speed_regulation(cvs, 0.0, 0.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
