/*! 
 * \author Group 2
 * \file path_regulation_gr2.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR2_H_
#define _PATH_REGULATION_GR2_H_

#include "CtrlStruct_gr2.h"
#include "namespace_ctrl.h"
#include "init_pos_gr2.h"
#include "odometry_gr2.h"
#include "opp_pos_gr2.h"
#include "triangulation_gr2.h"
#include "strategy_gr2.h"
NAMESPACE_INIT(ctrlGr2);

void follow_path(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
