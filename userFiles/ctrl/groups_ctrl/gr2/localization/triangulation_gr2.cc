#include "triangulation_gr2.h"
#include "useful_gr2.h"
#include "init_pos_gr2.h"
#include <math.h>
#include <Eigen/Dense>
NAMESPACE_INIT(ctrlGr2);

/*! \brief set the fixed beacons positions, depending on the team
 * 
 * \param[in] team_id ID of the team ('TEAM_A' or 'TEAM_B')
 * \param[out] x_beac_1 first beacon x position [m]
 * \param[out] y_beac_1 first beacon y position [m]
 * \param[out] x_beac_2 second beacon x position [m]
 * \param[out] y_beac_2 second beacon y position [m]
 * \param[out] x_beac_3 third beacon x position [m]
 * \param[out] y_beac_3 third beacon y position [m]
 *
 * This function can be adapted, depending on the map.
 */
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
	double *x_beac_2, double *y_beac_2, double *x_beac_3, double *y_beac_3)
{
	switch (team_id)
	{
		case TEAM_A:
			*x_beac_1 = 1.062;
			*y_beac_1 = 1.562;

			*x_beac_2 = -1.062;
			*y_beac_2 = 1.562;

			*x_beac_3 = 0.0;
			*y_beac_3 = -1.562;
			break;

		case TEAM_B:
			*x_beac_1 = 1.062;
			*y_beac_1 = -1.562;

			*x_beac_2 = -1.062;
			*y_beac_2 = -1.562;

			*x_beac_3 = 0.0;
			*y_beac_3 = 1.562;
			break;
	
		default:
			printf("Error unknown team ID (%d) !\n", team_id);
			exit(EXIT_FAILURE);
	}
}

/*! \brief get the index of the best angle prediction
 * 
 * \param[in] alpha_predicted angle to reach [rad]
 * \param[in] alpha_a angle computed for A [rad]
 * \param[in] alpha_b angle computed for B [rad]
 * \param[in] alpha_c angle computed for C [rad]
 * \return best index (0, 1, or 2)
 */
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c)
{
	double pred_err_a, pred_err_b, pred_err_c;

	pred_err_a = fabs(limit_angle(alpha_a - alpha_predicted));
	pred_err_b = fabs(limit_angle(alpha_b - alpha_predicted));
	pred_err_c = fabs(limit_angle(alpha_c - alpha_predicted));

	return (pred_err_a < pred_err_b) ? ((pred_err_a < pred_err_c) ? 0 : 2) : ((pred_err_b < pred_err_c) ? 1 : 2);
}

/*! \brief triangulation main algorithm
 * 
 * \param[in] cvs controller main structure
 *
 * computation found here: http://www.telecom.ulg.ac.be/triangulation/
 */
void triangulation(CtrlStruct *cvs)
{
	// variables declaration
	RobotPosition *pos_tri, *rob_pos;
	CtrlIn *inputs;
	double x_beacons[3];
	double y_beacons[3];
	double relatives_angles[3];

	int alpha_1_index, alpha_2_index, alpha_3_index;
	int rise_index_1, rise_index_2, rise_index_3;
	int fall_index_1, fall_index_2, fall_index_3;

	double alpha_a, alpha_b, alpha_c;
	double alpha_1, alpha_2, alpha_3;
	double alpha_1_predicted, alpha_2_predicted, alpha_3_predicted;
	double x_beac_1, y_beac_1, x_beac_2, y_beac_2, x_beac_3, y_beac_3;

	// variables initialization
	pos_tri = cvs->triang_pos;
	rob_pos = cvs->rob_pos;
	inputs  = cvs->inputs;

	// safety
	if ((inputs->rising_index_fixed < 0) || (inputs->falling_index_fixed < 0))
	{
		return;
	}

	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);	

	// indexes fot the angles detection
	rise_index_1 = inputs->rising_index_fixed;
	rise_index_2 = (rise_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_1 - 1;
	rise_index_3 = (rise_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : rise_index_2 - 1;

	fall_index_1 = inputs->falling_index_fixed;
	fall_index_2 = (fall_index_1 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_1 - 1;
	fall_index_3 = (fall_index_2 - 1 < 0) ? NB_STORE_EDGE-1 : fall_index_2 - 1;
	
	double last_rising_fixed[NB_STORE_EDGE];  ///< rotating list with the last rising edges detected [rad]
	double last_falling_fixed[NB_STORE_EDGE]; ///< rotating list with the last falling edges detected [rad]

	// beacons angles measured with the laser (to compute)
	alpha_a = inputs->last_rising_fixed[rise_index_1] + ((inputs->last_falling_fixed[fall_index_1] - inputs->last_rising_fixed[rise_index_1]  ) /2);
	alpha_b = inputs->last_rising_fixed[rise_index_2] + ((inputs->last_falling_fixed[fall_index_2] - inputs->last_rising_fixed[rise_index_2] )/2);
	alpha_c = inputs->last_rising_fixed[rise_index_3] + ((inputs->last_falling_fixed[fall_index_3] - inputs->last_rising_fixed[rise_index_3] )/2);

	// beacons angles predicted thanks to odometry measurements (to compute)
	alpha_1_predicted = limit_angle((rob_pos->theta + M_PI) + ((atan((y_beac_1 - rob_pos->y) / (x_beac_1 - rob_pos->x))))) ;
	alpha_2_predicted = limit_angle((3*M_PI/2 + rob_pos->theta) + (M_PI / 2 - atan((y_beac_2 - rob_pos->y) / (x_beac_2 - rob_pos->x)) )) ;
	alpha_3_predicted = rob_pos->x > 0 ? limit_angle((rob_pos->theta + 2*M_PI + ( atan((y_beac_3 - rob_pos->y) / (x_beac_3 - rob_pos->x))))) :
										limit_angle(rob_pos->theta + M_PI/2 + (M_PI/2 -  ( atan((y_beac_3 - rob_pos->y) / (x_beac_3 - rob_pos->x)))));

	// indexes of each beacon
	alpha_1_index = index_predicted(alpha_1_predicted, alpha_a, alpha_b, alpha_c);
	alpha_2_index = index_predicted(alpha_2_predicted, alpha_a, alpha_b, alpha_c);
	alpha_3_index = index_predicted(alpha_3_predicted, alpha_a, alpha_b, alpha_c);

	// safety
	if ((alpha_1_index == alpha_2_index) || (alpha_1_index == alpha_3_index) || (alpha_2_index == alpha_3_index))
	{
		return;
	}

	// angle of the first beacon
	switch (alpha_1_index)
	{
		case 0: alpha_1 = alpha_a; break;
		case 1: alpha_1 = alpha_b; break;
		case 2: alpha_1 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_1_index);
			exit(EXIT_FAILURE);
	}

	// angle of the second beacon
	switch (alpha_2_index)
	{
		case 0: alpha_2 = alpha_a; break;
		case 1: alpha_2 = alpha_b; break;
		case 2: alpha_2 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_2_index);
			exit(EXIT_FAILURE);
	}

	// angle of the third beacon
	switch (alpha_3_index)
	{
		case 0: alpha_3 = alpha_a; break;
		case 1: alpha_3 = alpha_b; break;
		case 2: alpha_3 = alpha_c; break;
	
		default:
			printf("Error: unknown index %d !\n", alpha_3_index);
			exit(EXIT_FAILURE);
	}
	x_beacons[0] = x_beac_1;
	x_beacons[1] = x_beac_2;
	x_beacons[2] = x_beac_3;

	y_beacons[0] = y_beac_1;
	y_beacons[1] = y_beac_2;
	y_beacons[2] = y_beac_3;

	relatives_angles[0] = alpha_1;
	relatives_angles[1] = alpha_2;
	relatives_angles[2] = alpha_3;

	// ----- triangulation computation start ----- //
/*
	Eigen::Matrix<float, 2, 2> A;
	Eigen::Matrix<float, 2, 1> b;
	A << compute(Operation::X, 0, 1, x_beacons, y_beacons, relatives_angles) - compute(Operation::X, 1, 2, x_beacons, y_beacons, relatives_angles),
		compute(Operation::Y, 0, 1, x_beacons, y_beacons, relatives_angles) - compute(Operation::Y, 1, 2, x_beacons, y_beacons, relatives_angles),
		compute(Operation::X, 1, 2, x_beacons, y_beacons, relatives_angles) - compute(Operation::X, 2, 0, x_beacons, y_beacons, relatives_angles),
		compute(Operation::Y, 1, 2, x_beacons, y_beacons, relatives_angles) - compute(Operation::Y, 2, 0, x_beacons, y_beacons, relatives_angles);

	b << compute(Operation::K, 0, 1, x_beacons, y_beacons, relatives_angles) - compute(Operation::K, 1, 2, x_beacons, y_beacons, relatives_angles),
		compute(Operation::K, 1, 2, x_beacons, y_beacons, relatives_angles) - compute(Operation::K, 2, 0, x_beacons, y_beacons, relatives_angles);

	Eigen::Vector2f position = A.colPivHouseholderQr().solve(b);*/
	// robot position
	double *positions = compute(x_beacons, y_beacons, relatives_angles)()->get_positions();
	const double beacon_uncentered = 0.083; // beacon not at the center of the robot
	pos_tri->x = positions[0] + beacon_uncentered * cos(rob_pos->theta);
	pos_tri->y = positions[1] - beacon_uncentered * sin(rob_pos->theta);

	// robot orientation
	pos_tri->theta = rob_pos->theta;

	// ----- triangulation computation end ----- //
}
//double compute(Operation op, int i, int j, double *x_beacons, double *y_beacons, double *angles){
//	switch (op) {
//	case Beacon_coord :
//		return;
//	case K :
//		return;
//	case D :
//		return;
//	case Circle_center :
//		return;
//	case Cot :
//		return (angles[j] - angles[i] == 0 ? pow(10,8) : (fmod((angles[j] - angles[i]), M_PI) == 0.0 ? pow(-10, 8) : 1/tan(angles[j] - angles[i])));
//	}
//}
NAMESPACE_CLOSE();
