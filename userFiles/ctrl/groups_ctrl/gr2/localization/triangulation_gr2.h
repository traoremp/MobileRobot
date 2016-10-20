/*! 
 * \author Group 2
 * \file triangulation_gr2.h
 * \brief triangulation to get the robot abgrXute position
 */

#ifndef _TRIANGULATION_GR2_H_
#define _TRIANGULATION_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"
#include <stdexcept>
NAMESPACE_INIT(ctrlGr2);
typedef enum Operation {Beacon_coord, Cot, Circle_center, K, D};
void fixed_beacon_positions(int team_id, double *x_beac_1, double *y_beac_1,
	double *x_beac_2, double *y_beac_2, double *x_beac_3, double *y_beac_3);
int index_predicted(double alpha_predicted, double alpha_a, double alpha_b, double alpha_c);
void triangulation(CtrlStruct *cvs);

class compute {
public:
	compute(double *x_beacons, double *y_beacons, double * angles) {
		x_beacons_ = x_beacons;
		y_beacons_ = y_beacons;
		angles_ = angles;
	}
	double* get_positions() {
		return positions_;
	}
	compute* operator()() {
		//new beacon positions
		double beac_pos_x_1;
		double beac_pos_y_1;		
		double beac_pos_x_3;
		double beac_pos_y_3;
		//cotangentes
		double cot_1_2, cot_2_3, cot_3_1;
		//circle centers
		double center_X_1_2, center_Y_1_2, center_X_2_3, center_Y_2_3, center_X_3_1, center_Y_3_1;
		//K31
		double K_3_1;
		//D
		double D; 

		//compute beacon modified positions
		beac_pos_x_1 = x_beacons_[0] - x_beacons_[1];
		beac_pos_y_1 = y_beacons_[0] - y_beacons_[1];

		beac_pos_x_3 = x_beacons_[2] - x_beacons_[1];
		beac_pos_y_3 = y_beacons_[2] - y_beacons_[1];

		//compute the three cot()
		cot_1_2 = (angles_[1] - angles_[0] == 0 ? pow(10, 8) : (fmod((angles_[1] - angles_[0]), M_PI) == 0.0 ? pow(-10, 8) : 1 / tan(angles_[1] - angles_[0])));
		cot_2_3 = (angles_[2] - angles_[1] == 0 ? pow(10, 8) : (fmod((angles_[2] - angles_[1]), M_PI) == 0.0 ? pow(-10, 8) : 1 / tan(angles_[2] - angles_[1])));
		cot_3_1 = (1 - (cot_1_2 * cot_2_3)) / (cot_1_2 + cot_2_3);

		//compute modified circle center
		center_X_1_2 = beac_pos_x_1 + cot_1_2 * beac_pos_y_1;
		center_Y_1_2 = beac_pos_y_1 - cot_1_2 * beac_pos_x_1;

		center_X_2_3 = beac_pos_x_3 - cot_2_3 * beac_pos_y_3;
		center_Y_2_3 = beac_pos_y_3 + cot_2_3 * beac_pos_x_3;

		center_X_3_1 = (beac_pos_x_3 + beac_pos_x_1) + cot_3_1 * (beac_pos_y_3 - beac_pos_y_1);
		center_Y_3_1 = (beac_pos_y_3 + beac_pos_y_1) - cot_3_1 * (beac_pos_x_3 - beac_pos_x_1);

		//compute K31
		K_3_1 = (beac_pos_x_1 * beac_pos_x_3) + (beac_pos_y_1 * beac_pos_y_3) + cot_3_1*(beac_pos_x_1*beac_pos_y_3 - beac_pos_x_3*beac_pos_y_1);

		//compute D
		D = (center_X_1_2 - center_X_2_3) * (center_Y_2_3 - center_Y_3_1) - (center_Y_1_2 - center_Y_2_3) * (center_X_2_3 - center_X_3_1);
		if (D == 0)
			throw std::domain_error("D = 0");
		positions_[0] = x_beacons_[1] + (K_3_1 * (center_Y_1_2 - center_Y_2_3) / D);
		positions_[1] = y_beacons_[1] + (K_3_1 * (center_X_2_3 - center_X_1_2) / D);

		return this;
	}
private:
	double positions_[2];
	double *x_beacons_;
	double *y_beacons_;
	double *angles_;
};

NAMESPACE_CLOSE();

#endif
