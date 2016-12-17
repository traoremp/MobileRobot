#include "opp_pos_gr2.h"
#include "init_pos_gr2.h"
#include "useful_gr2.h"
#include <math.h>
#include <iostream>
NAMESPACE_INIT(ctrlGr2);

/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp;
	int rise_index_1, rise_index_2, fall_index_1, fall_index_2;

	double delta_t;
	double rise_1, rise_2, fall_1, fall_2;

	CtrlIn *inputs;
	RobotPosition *rob_pos;
	OpponentsPosition *opp_pos;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;

	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	// low pass filter time increment ('delta_t' is the last argument of the 'first_order_filter' function)
	delta_t = inputs->t - opp_pos->last_t;
	opp_pos->last_t = inputs->t;

	// indexes
	rise_index_1 = inputs->rising_index;
	fall_index_1 = inputs->falling_index;

	// rise and fall angles of the first opponent
	rise_1 = inputs->last_rising[rise_index_1];
	fall_1 = inputs->last_falling[fall_index_1];

	// rise and fall angles of the second opponent
	if (nb_opp == 2)
	{
		rise_index_2 = (rise_index_1-1 < 0) ? NB_STORE_EDGE-1 : rise_index_1-1;
		fall_index_2 = (fall_index_1-1 < 0) ? NB_STORE_EDGE-1 : fall_index_1-1;

		rise_2 = inputs->last_rising[rise_index_2];
		fall_2 = inputs->last_falling[fall_index_2];
	}

	// ----- opponents position computation start ----- //

	opp_pos->x[0] = 0.0;
	opp_pos->y[0] = 0.0;	
	single_opp_tower(rise_1, fall_1, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_pos->x[0], &opp_pos->y[0]);

	if(nb_opp == 2){
		opp_pos->x[1] = 0.0;
		opp_pos->y[1] = 0.0;
		single_opp_tower(rise_2, fall_2, rob_pos->x, rob_pos->y, rob_pos->theta, &opp_pos->x[1], &opp_pos->y[1]);
	}
	
	
	
	// ----- opponents position computation end ----- //
}

/*! \brief compute a single opponent position
 * 
 * \param[in] last_rise last rise relative angle [rad]
 * \param[in] last_fall last fall relative angle [rad]
 * \param[in] rob_x robot x position [m]
 * \param[in] rob_y robot y position [m]
 * \param[in] rob_theta robot orientation [rad]
 * \param[out] new_x_opp new known x opponent position
 * \param[out] new_y_opp new known y opponent position
 * \return 1 if computation successful, 0 otherwise
 */
int single_opp_tower(double last_rise, double last_fall, double rob_x, double rob_y, double rob_theta, double *new_x_opp, double *new_y_opp)
{
	//*new_x_opp = 0.0;
	//*new_y_opp = 0.0;

	double dist;
	const double r_beacon = 0.04; //radius beacon
	const double beacon_uncentered = 0.083; // beacon not at the center of the robot

	dist = r_beacon/sin((last_fall-last_rise)/2); // distance from other beacon center

	*new_x_opp = rob_x + (beacon_uncentered*cos(rob_theta )) + dist*cos(rob_theta + (last_fall+last_rise)/2);
	*new_y_opp = rob_y + (beacon_uncentered*sin(rob_theta )) + dist*sin(rob_theta + (last_fall+last_rise)/2);
	
	return 1;
}

/*! \brief check if there is an opponent in front of the robot
 * 
 * \param[in] cvs controller main structure
 * \return 1 if opponent robot in front of the current robot
 */
int check_opp_front(CtrlStruct *cvs)
{
	// variables declaration
	int i, nb_opp;
	double k; //coefficient qui satisfait l'equation
	
	OpponentsPosition *opp_pos;
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
	nb_opp = opp_pos->nb_opp;

	// no opponent
	if (!nb_opp)
	{
		return 0;
	}

	// safety
	if (nb_opp < 0 || nb_opp > 2)
	{
		printf("Error: number of opponents cannot be %d!\n", nb_opp);
		exit(EXIT_FAILURE);
	}

	for(i=0; i<nb_opp; i++)
	{
		// ----- opponents check computation start ----- //
		return int(isInFront(rob_pos->x, rob_pos->y, rob_pos->theta, opp_pos->x[i], opp_pos->y[i]));
		// ----- opponents check computation end ----- //
	}

	return 0;
}

bool isInFront(double rob_x, double rob_y, double rob_theta, double opp_x, double opp_y ){
	double k; //Valeur qui satisfait l'equation
	double vector_x, vector_y;
	//double angle_in_positive_range = (rob_theta < 0 ? rob_theta + 2*M_PI : rob_theta);
	vector_x = cos(rob_theta);
	vector_y = sin(rob_theta);
	
	k = (opp_x - rob_x) / vector_x;
	return (rob_y + k * vector_y == opp_y && k > 0);
}
NAMESPACE_CLOSE();
