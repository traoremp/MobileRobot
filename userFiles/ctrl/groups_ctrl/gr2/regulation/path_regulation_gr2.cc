#include "path_regulation_gr2.h"
#include "path_planning_gr2.h"
#include "useful_gr2.h"
#include "speed_regulation_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{
	PathPlanning* path = cvs->path;
	Map_Element rob_pos;
	Map_Element vector;
	double k = 0;
	double theta_goal;
	double x = cvs->rob_pos->x;
	double y = cvs->rob_pos->y;
	double theta = cvs->rob_pos->theta;
	static int forward;
	int ratio = 100;

	if (!path->shortest_path->empty()) {
		Map_Element node = path->shortest_path->front();
		double dist = (node[0] - x) *(node[0] - x) + (node[1] - y)*(node[1] - y) ;

		theta_goal = -1 * atan2((node[0] - x), (node[1] - y)) + M_PI_2;
		theta_goal = limit_angle(theta_goal);
		//set_plot(theta_goal, "theta_goal[rad]");
		//set_plot(forward, "forward");

		if (((theta_goal - theta < 0.02) && (theta_goal - theta > -0.02)) || ((theta - theta_goal < 0.02) && (theta - theta_goal > 0.02)) && !forward)
		{
			forward = 1;
			speed_regulation(cvs, 25, 25);
		}
		else if ((theta_goal < theta) && forward == 0)
			speed_regulation(cvs, -25, 25);
		else if ((theta_goal > theta) && forward == 0)
			speed_regulation(cvs, 25, -25);


		//printf("%f %f %f %f %f %f\n", node[0], node[1], x, y, theta_goal, theta);

		if (((node[0] - x < 0.04 && node[0] - x > -0.04) || (x - node[0] < 0.04 && x - node[0] > -0.04))
			&&
			((node[1] - y < 0.04 && node[1] - y > -0.04) || (y - node[1] < 0.04 && y - node[1] > -0.04)))
		{
			path->shortest_path->pop_front();
			forward = 0;
		}
	}
	else
		cvs->strat->main_state++;

}

NAMESPACE_CLOSE();
