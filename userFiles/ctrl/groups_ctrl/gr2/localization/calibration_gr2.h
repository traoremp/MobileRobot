/*! 
 * \author Group 2
 * \file calibration_gr2.h
 * \brief calibration of the robot
 */

#ifndef _CALIBRATION_GR2_H_
#define _CALIBRATION_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/// robot calibration
typedef struct RobotCalibration
{
	double t_flag; ///< time for flag

	int flag; ///< flag for calibration

	double wall_Y;
	double wall_X;

} RobotCalibration;

void calibration(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
