#include <assert.h>
#include "kalman_gr2.h"
#include "odometry_gr2.h"
#include "triangulation_gr2.h"
#include "useful_gr2.h"
NAMESPACE_INIT(ctrlGr2);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void kalman(CtrlStruct *cvs, double* angles_beacons)
{
	static KalmanStruc* my_filter = new KalmanStruc();
	// variable declaration
	RobotPosition *rob_pos;
	RobotPosition *triang_pos;

	CtrlIn *inputs;
	double r_sp, l_sp;
	double dt;

	double ds_r, ds_l, ds, dtheta;

	double x_beac_1, y_beac_1, x_beac_2, y_beac_2, x_beac_3, y_beac_3;
	// variables initialization
	// known positions of the beacons
	fixed_beacon_positions(cvs->team_id, &x_beac_1, &y_beac_1, &x_beac_2, &y_beac_2, &x_beac_3, &y_beac_3);

	const double radius_w = 0.03;
	const double dist_w = 0.225;
	const double beacon_uncentered = 0.083; // beacon not at the center of the robot

	rob_pos = cvs->rob_pos;
	triang_pos = cvs->triang_pos;
	inputs = cvs->inputs;
	// time
	dt = inputs->t - rob_pos->last_t; // time increment since last call

	//recreer les variables d'odometry
	r_sp = inputs->r_wheel_speed; // right wheel speed
	l_sp = inputs->l_wheel_speed; // left wheel speed
	ds_r = r_sp*radius_w*dt;
	ds_l = l_sp*radius_w*dt;

	ds = (ds_r + ds_l) / 2;
	dtheta = (ds_r - ds_l) / dist_w;
	
	
	//control variable matrix
	Eigen::Vector2f u_k;
	u_k << ds, dtheta;

	//error vector
	Eigen::Vector2f W;
	W << WHEEL_NOISE,
		WHEEL_NOISE;

	//coefficient matrix to the error vector
	Eigen::Matrix<float, 3, 2> G_k;
	G_k << cos(rob_pos->theta + (u_k[1] / 2)) - (u_k[0] / dist_w)*sin(rob_pos->theta + (u_k[1] / 2)), cos(rob_pos->theta + (u_k[1] / 2)) + (u_k[0] / dist_w)*sin(rob_pos->theta + (u_k[1] / 2)),
		sin(rob_pos->theta + (u_k[1] / 2)) + (u_k[0] / dist_w)*cos(rob_pos->theta + (u_k[1] / 2)), sin(rob_pos->theta + (u_k[1] / 2)) - (u_k[0] / dist_w)*cos(rob_pos->theta + (u_k[1] / 2)),
		2 / dist_w, 2 / dist_w;
	G_k /= 2;
	
	//Next state estimate
	Eigen::Vector3f next_x;
	next_x << rob_pos->x,
		rob_pos->y,
		rob_pos->theta;
	next_x += G_k * W;

	//error covariance
	Eigen::Matrix2f cov_w;
	cov_w << pow(WHEEL_NOISE, 2), 0.0,
		0.0, pow(WHEEL_NOISE, 2);

	
	//robot position without angle in vector
	Eigen::Vector3f robot_pos;
	robot_pos << rob_pos->x, rob_pos->y,0.0;

	//unit vector pointing in direction of the robot
	Eigen::Vector3f U;
	U << cos(rob_pos->theta), sin(rob_pos->theta), 0.0;

	//Angles estimates to get x in the coordinates of the measurements -> run x through measurement function h
	Eigen::Vector3f Y_estimate;
	Y_estimate << limit_angle(atan2((U.cross(Eigen::Vector3f(x_beac_1, y_beac_1,0.0) - robot_pos)).norm(), U.dot(Eigen::Vector3f(x_beac_1, y_beac_1,0.0) - robot_pos) + beacon_uncentered)),
		limit_angle(atan2((U.cross(Eigen::Vector3f(x_beac_2, y_beac_2,0.0) - robot_pos)).norm(), U.dot(Eigen::Vector3f(x_beac_2, y_beac_2,0.0) - robot_pos) + beacon_uncentered)),
		limit_angle(atan2((U.cross(Eigen::Vector3f(x_beac_3, y_beac_3,0.0) - robot_pos)).norm(), U.dot(Eigen::Vector3f(x_beac_3, y_beac_3,0.0) - robot_pos) + beacon_uncentered));
	//matrix R coivariances du bruit sur les mesures d'angles
	Eigen::Matrix3f R;
	R << pow(TOWER_NOISE, 2), 0.0, 0.0,
		0.0, pow(TOWER_NOISE, 2), 0.0,
		0.0, 0.0, pow(TOWER_NOISE, 2);
	//measured angles
	Eigen::Vector3f Y_measured;
	Y_measured << angles_beacons[0],
		angles_beacons[1],
		angles_beacons[2];

	//Process error covariance matrix 
	Eigen::Matrix3f Q_k;
	Q_k = G_k * cov_w * G_k.transpose();

	//Jacobian of the process function f
	Eigen::Matrix3f F_x;
	F_x << 1.0, 0.0, -u_k[0] * sin(rob_pos->theta + (u_k[1] / 2)),
		0.0, 1.0, u_k[0] * cos(rob_pos->theta + (u_k[1] / 2)),
		0.0, 0.0, 1.0;
	//compute delta relative to each beacons
	double delta_1 = pow(((x_beac_1 - rob_pos->x)*cos(rob_pos->theta) + (y_beac_1 - rob_pos->y)*sin(rob_pos->theta)), 2)
		+ pow(((y_beac_1 - rob_pos->y)*cos(rob_pos->theta) + (x_beac_1 - rob_pos->x)*sin(rob_pos->theta)), 2);
	double delta_2 = pow(((x_beac_2 - rob_pos->x)*cos(rob_pos->theta) + (y_beac_2 - rob_pos->y)*sin(rob_pos->theta)), 2)
		+ pow(((y_beac_2 - rob_pos->y)*cos(rob_pos->theta) + (x_beac_2 - rob_pos->x)*sin(rob_pos->theta)), 2);
	double delta_3 = pow(((x_beac_3 - rob_pos->x)*cos(rob_pos->theta) + (y_beac_3 - rob_pos->y)*sin(rob_pos->theta)), 2)
		+ pow(((y_beac_3 - rob_pos->y)*cos(rob_pos->theta) + (x_beac_3 - rob_pos->x)*sin(rob_pos->theta)), 2);
	//H_x jacobian of the measurement function h
	Eigen::Matrix3f H_x;
	H_x << (y_beac_1 - rob_pos->y) / delta_1, (y_beac_2 - rob_pos->y) / delta_2, (y_beac_3 - rob_pos->y) / delta_3,
		(x_beac_1 - rob_pos->y) / delta_1, (x_beac_2 - rob_pos->y) / delta_2, (x_beac_3 - rob_pos->y) / delta_3,
		(pow(y_beac_1 - rob_pos->y, 2) + pow(x_beac_1 - rob_pos->x, 2)) / delta_1, (pow(y_beac_2 - rob_pos->y, 2) + pow(x_beac_2 - rob_pos->x, 2)) / delta_2, (pow(y_beac_3 - rob_pos->y, 2) + pow(x_beac_3 - rob_pos->x, 2)) / delta_3;
	
	//prediction on the next covariance matrix  P k + 1 predicted
	Eigen::Matrix3f next_x_cov_mat_p;
	next_x_cov_mat_p = F_x * my_filter->X_cov_mat * F_x.transpose() + Q_k;
	
	//compute S(k+1)
	Eigen::Matrix3f S_k = H_x * next_x_cov_mat_p * H_x.transpose() + R;

	//compute kalman gain 
	Eigen::Matrix3f kalman_gain = (next_x_cov_mat_p * H_x.transpose()) * S_k.inverse();
	//adjust the estimate
	next_x = next_x + (kalman_gain *(Y_measured - Y_estimate));
	rob_pos->x = next_x[0];
	rob_pos->y = next_x[1];
	rob_pos->theta = next_x[2];
	
	//adjust error on the estimate
	my_filter->X_cov_mat = next_x_cov_mat_p - (kalman_gain * S_k *kalman_gain.transpose());
}

KalmanStruc::KalmanStruc() {
	X_cov_mat << pow(T1_UNCERT, 2), 0.0, 0.0,
		0.0, pow(T2_UNCERT, 2), 0.0,
		0.0, 0.0, pow(DEG_TO_RAD*R3_UNCERT, 2)
		;
}

NAMESPACE_CLOSE();
