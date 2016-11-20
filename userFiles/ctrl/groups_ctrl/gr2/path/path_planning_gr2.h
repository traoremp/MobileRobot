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
#include <list>
#include <Eigen/Dense>


#define Map_Element Eigen::Vector2d //first -> X, second -> Y
NAMESPACE_INIT(ctrlGr2);

class Obstacle{
	public: 
		Obstacle() = default;
		Obstacle(std::list<std::shared_ptr<Map_Element>> points);
		inline void add_point(Map_Element& point) { 
			points_.push_back(std::make_shared<Map_Element>(point));
		};
		inline std::list<std::shared_ptr<Map_Element>> getPoints() { return points_; }
	private : 
		std::list<std::shared_ptr<Map_Element>> points_; 
};
class TreeNode{
	public:
		TreeNode(Map_Element pos):position_(pos){};
		void add_child(std::unique_ptr<std::pair<int, TreeNode>>);
		inline Map_Element getPosition() { return position_; };
	private:
		//std::unique_ptr<TreeNode> parent_;
		std::vector<std::unique_ptr<std::pair<int, TreeNode>>> children_;
		Map_Element position_;
};
class PathTree{
	public:
		PathTree() = default;
		inline PathTree(std::unique_ptr<TreeNode> root, std::unique_ptr<TreeNode> destination) {
			root_ = std::move(root);
			target_Node_ = std::move(destination);
		}

	private:
		std::unique_ptr<TreeNode> root_;
		std::unique_ptr<TreeNode> target_Node_;
};
/// path-planning main structure
struct PathPlanning
{
	void init_tree(Map_Element rob_pos, Map_Element destination);
	bool isConnectable( Map_Element, Map_Element);

	std::unique_ptr<std::vector<Map_Element>> vertices_;
	std::unique_ptr<std::vector<Obstacle>> obstacles_;
	std::unique_ptr<PathTree> tree_;
	//int dummy_variable; ///< put your own variable, this is just an example without purpose
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
