/*! 
 * \author Group 2
 * \file triangulation_gr2.h
 * \brief triangulation to get the robot abgrXute position
 */

#ifndef _TRIANGULATION_GR2_H_
#define _TRIANGULATION_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"

NAMESPACE_INIT(ctrlGr2);
typedef enum Operation {K, X, Y, R, T};
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
	double *x_beac_2, double *y_beac_2, double *x_beac_3, double *y_beac_3);
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c);
void triangulation(CtrlStruct *cvs);
double compute(Operation op,int i, int j, double *x_beacons, double *y_beacons, double *anlges);

NAMESPACE_CLOSE();

#endif
