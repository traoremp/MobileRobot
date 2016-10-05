/*! 
 * \author Group 2
 * \file odometry_gr2.h
 * \brief odometry of the robot
 */

#ifndef _ODOMETRY_GR2_H_
#define _ODOMETRY_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"

NAMESPACE_INIT(ctrlGr2);

void update_odometry(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
