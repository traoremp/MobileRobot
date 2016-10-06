#include "odometry_gr2.h"
#include "useful_gr2.h"
#include "init_pos_gr2.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr2);

/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */
void update_odometry(CtrlStruct *cvs)
{
	// variables declaration
	double r_sp, l_sp;
	double dt;

	const double radius_w = 0.03;
	const double dist_w = 0.225;

	double ds_r, ds_l, ds, dtheta;

	RobotPosition *rob_pos;
	CtrlIn *inputs;

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed

	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

	// safety
	if (dt <= 0.0)
	{
		return;
	}

	// ----- odometry computation start ----- //

	ds_r = r_sp*radius_w*dt;
	ds_l = l_sp*radius_w*dt;

	ds = (ds_r+ds_l)/2;
	dtheta = (ds_r-ds_l)/dist_w;

	rob_pos->x += ds*cos(rob_pos->theta + dtheta/2);
	rob_pos->y += ds*sin(rob_pos->theta + dtheta/2);
	rob_pos->theta += dtheta;

	// ----- odometry computation end ----- //

	// last update time
	rob_pos->last_t = inputs->t;
}

NAMESPACE_CLOSE();
