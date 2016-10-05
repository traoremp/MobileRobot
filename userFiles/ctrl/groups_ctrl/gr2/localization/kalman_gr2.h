/*! 
 * \author Group 2
 * \file kalman_gr2.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR2_H_
#define _KALMAN_GR2_H_

#include "CtrlStruct_gr2.h"
#include "init_pos_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/// Kalman main structure
struct KalmanStruc
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void kalman(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
