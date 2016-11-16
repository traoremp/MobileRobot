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
NAMESPACE_INIT(ctrlGr2);
class PathTree{
	public:
		PathTree();

	private:
		std::unique_ptr<TreeNode> root_;
		std::unique_ptr<TreeNode> target_Node_;
}
class TreeNode{
	public:
		TreeNode();
	private:
		std::unique_ptr<TreeNode> parent_;
		vector<std::unique_ptr<std::pair<int, TreeNode>>> children_;
}
/// path-planning main structure
struct PathPlanning
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
