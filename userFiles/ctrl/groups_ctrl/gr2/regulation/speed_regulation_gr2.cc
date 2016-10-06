#include "speed_regulation_gr2.h"
#include "useful_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief wheel speed regulation
 * 
 * \param[in,out] cvs controller main structure
 * \parem[in] r_sp_ref right wheel speed reference [rad/s]
 * \parem[in] l_sp_ref left wheel speed reference [rad/s]
 */
 //pu.bai@epfl.ch
void speed_regulation(CtrlStruct *cvs, double r_sp_ref, double l_sp_ref)
{
	double r_sp, l_sp;
	double dt;
	double kp = 80;
	double ki = 15;
	double output_r_wheel, output_l_wheel;
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
	sp_reg->int_error_r = sp_reg->int_error_r + ((r_sp_ref - r_sp) * dt);
	sp_reg->int_error_l = sp_reg->int_error_l + ((l_sp_ref - l_sp) * dt);

	output_r_wheel = kp*(r_sp_ref - r_sp) + ki * limit_range(sp_reg->int_error_r, -10, 10);
	output_l_wheel = kp*(l_sp_ref - l_sp) + ki * limit_range(sp_reg->int_error_l, -10, 10);
	
	
	// wheel commands
	outputs->wheel_commands[R_ID] = output_r_wheel;
	outputs->wheel_commands[L_ID] = output_l_wheel;

	// ----- Wheels regulation computation end ----- //

	// last update time
	sp_reg->last_t = inputs->t;

}

NAMESPACE_CLOSE();
