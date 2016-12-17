/*! 
 * \author Group 2
 * \file strategy_gr2.h
 * \brief strategy during the game
 */

#ifndef _STRATEGY_GR2_H_
#define _STRATEGY_GR2_H_

#include "CtrlStruct_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/// strategy main structure
typedef struct Strategy
{
	int main_state; ///< main state of the strategy
	
} Strategy;

/// 'main_state' states (adapt with your own states)
enum {GAME_STATE_A, GAME_STATE_B, GAME_STATE_C, GAME_STATE_D, GAME_STATE_E, GAME_STATE_F, GAME_STATE_G, GAME_STATE_H, GAME_STATE_I, GAME_STATE_J, GAME_STATE_K, GAME_STATE_L, GAME_STATE_M, GAME_STATE_N, GAME_STATE_O, GAME_STATE_P, GAME_STATE_Q, GAME_STATE_R};

Strategy* init_strategy();
void free_strategy(Strategy *strat);
void main_strategy(CtrlStruct *cvs);
void goToBase(CtrlStruct *cvs);
void getOutofBase(CtrlStruct *cvs);
void getOutofStart(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
