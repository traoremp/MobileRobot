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
	Map_Element start = Map_Element(cvs->rob_pos->x, cvs->rob_pos->y);
	Map_Element goal = Map_Element(-0.8, 0.0);
	//path->init_tree();
	path->AStar(start, goal);
	Map_Element rob_pos;
	Map_Element vector;
	double k = 0;
	while (!path->shortest_path->empty()) {
		//Map_Element node = path->shortest_path->front();
		//speed_regulation(cvs, -10.0, 10.0);
		//while (!(rob_pos[1]+ k * vector[1]== node[1] && k >= 0))
		//{
		//	rob_pos = Map_Element(cvs->rob_pos->x, cvs->rob_pos->y);
		//	//double k; //Valeur qui satisfait l'equation
		//	//double vector_x, vector_y;
		//	////double angle_in_positive_range = (rob_theta < 0 ? rob_theta + 2*M_PI : rob_theta);
		//	//vector_x = cos(rob_theta);
		//	//vector_y = sin(rob_theta);

		//	//k = (opp_x - rob_x) / vector_x;
		//	//return (rob_y + k * vector_y == opp_y && k > 0);
		//	
		//	vector = Map_Element(cos(cvs->rob_pos->theta), sin(cvs->rob_pos->theta));
		//	k = (node[0] - rob_pos[0]) / vector[0];
		//	
		//}
		//while (Map_Element(cvs->rob_pos->x, cvs->rob_pos->y) != node) {
		//	speed_regulation(cvs, 10.0, 10.0);
		//}
		
		//path->shortest_path->pop_front();
			
	}

}

NAMESPACE_CLOSE();
