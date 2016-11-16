/*! 
 * \author Group 2
 * \file path_planning_gr2.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR2_H_
#define _PATH_PLANNING_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"
#include <memory>
#include <vector>
#include <map>
#include <set>
NAMESPACE_INIT(ctrlGr2);
class Map_Element {
	public: 
		Map_Element(double, double);
	private: 
		double position_x_;
		double position_y_;
}
class Obstacle{
	public: 
		Obstacle(std::unique_ptr<std::List> points);
	private : 
		std::unique_ptr<std::List<std::unique_ptr<Map_Element>>> points_; 
}
class TreeNode{
	public:
		TreeNode();
	private:
		std::unique_ptr<TreeNode> parent_;
		vector<std::unique_ptr<std::pair<int, TreeNode>>> children_;
		Map_Element position_;
}
class PathTree{
	public:
		PathTree();

	private:
		std::unique_ptr<TreeNode> root_;
		std::unique_ptr<TreeNode> target_Node_;
}
/// path-planning main structure
struct PathPlanning
{
	std::unique_ptr<std::set<Obstacle>> obstacles_;
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
