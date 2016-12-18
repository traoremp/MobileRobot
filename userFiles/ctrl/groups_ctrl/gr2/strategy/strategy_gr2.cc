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
	static int init;
	PathPlanning* path;
	Map_Element start;
	Map_Element goal;

	// variables initialization
	strat  = cvs->strat;
	inputs = cvs->inputs;
	path = cvs->path;
	start = Map_Element(cvs->rob_pos->x, cvs->rob_pos->y);

	switch (strat->main_state)
	{
		case GAME_STATE_A:
			goal = Map_Element(-0.8, 0.0);
			path->AStar(start, goal);
			cvs->strat->main_state = GAME_STATE_B;
			break;

		case GAME_STATE_B:
			follow_path(cvs);
			break;

		case GAME_STATE_C:
			goal = Map_Element(-0.4, -0.6);
			path->AStar(start, goal);
			cvs->strat->main_state = GAME_STATE_D;
			break;

		case GAME_STATE_D:
			follow_path(cvs);
			break;

		case GAME_STATE_E:
			goal = Map_Element(0.7, -0.6);
			path->AStar(start, goal);
			cvs->strat->main_state = GAME_STATE_F;
			break;

		case GAME_STATE_F:
			follow_path(cvs);
			break;
		case GAME_STATE_G:
			speed_regulation(cvs, -100.0, 100.0);
			break;

		default:
			printf("Error: unknown strategy main state: %d !\n", strat->main_state);
			exit(EXIT_FAILURE);
	}
}

NAMESPACE_CLOSE();
