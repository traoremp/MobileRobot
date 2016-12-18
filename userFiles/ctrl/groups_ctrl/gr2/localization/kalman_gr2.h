/*! 
 * \author Group 2
 * \file kalman_gr2.h
 * \brief localization sensors fusion with Kalman
 */

#ifndef _KALMAN_GR2_H_
#define _KALMAN_GR2_H_

#include "CtrlStruct_gr2.h"
#include "init_pos_gr2.h"
#include "config_file.h"
#include <Eigen/Dense>
NAMESPACE_INIT(ctrlGr2);
#define DEG_TO_RAD (M_PI / 180)

/// Kalman main structure
struct KalmanStruc
{
	KalmanStruc();
	//Eigen::Vector3f X_current; next x sera calculee par odometry donc pas besoin de garder cette reference
	Eigen::Matrix3f X_cov_mat;//process covariance matrix

	//int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void kalman(CtrlStruct *cvs, double* );

NAMESPACE_CLOSE();

#endif
