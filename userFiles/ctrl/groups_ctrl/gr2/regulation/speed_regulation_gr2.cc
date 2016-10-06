#include "speed_regulation_gr2.h"
#include "useful_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief wheel speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
	double dt;

	double kp, ki, r_err, l_err,int_term_r, int_term_l, u_r, u_l;

	// variables declaration
	CtrlIn *inputs;
	CtrlOut *outputs;
	SpeedRegulation *sp_reg;

	// variables initialization
	inputs  = cvs->inputs;
	outputs = cvs->outputs;
	sp_reg  = cvs->sp_reg;

	// wheel speeds
	r_sp = inputs->r_wheel_speed;
	l_sp = inputs->l_wheel_speed;

	// time
	dt = inputs->t - sp_reg->last_t; // time interval since last call

	// ----- Wheels regulation computation start ----- //
	kp = 10;
	ki = 2;

	r_err = r_sp_ref - r_sp;
	l_err = l_sp_ref - l_sp;

	sp_reg->int_error_r += r_err;
	sp_reg->int_error_l += l_err;

	sp_reg->int_error_r = limit_range(sp_reg->int_error_r, -10, 10);
	sp_reg->int_error_l = limit_range(sp_reg->int_error_l, -10, 10);


	int_term_r = ki*sp_reg->int_error_r;
	int_term_l = ki*sp_reg->int_error_l;
	

	u_r = kp*r_err + int_term_r*dt;
	u_l = kp*l_err + int_term_l*dt;

	// wheel commands
	outputs->wheel_commands[R_ID] = u_r;
	outputs->wheel_commands[L_ID] = u_l;

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;
}

NAMESPACE_CLOSE();
