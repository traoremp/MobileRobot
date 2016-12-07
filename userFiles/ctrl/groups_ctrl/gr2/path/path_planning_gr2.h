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
#include <stack>
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
		TreeNode(TreeNode&& node) = default;
		TreeNode(const TreeNode& node) = default;		
		
		void add_child(std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>>);
		inline Map_Element getPosition() { return position_; };
		inline std::vector<std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>>>& getChildren() { return children_; }
		inline Map_Element getPosition() const { return position_; };
		inline const std::vector<std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>>>& getChildren() const { return children_; }
	private:
		
		std::vector<std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>>> children_;
		Map_Element position_;
};
class PathTree{
	public:
		PathTree() = default;
		inline PathTree(std::shared_ptr<TreeNode> root, std::shared_ptr<TreeNode> destination) {
			root_ = root;
			target_Node_ = destination;
		}
		inline std::shared_ptr<TreeNode> getRoot() { return root_; }
		inline std::shared_ptr<TreeNode> getTarget() { return target_Node_; }
	private:
		std::shared_ptr<TreeNode> root_;
		std::shared_ptr<TreeNode> target_Node_;
};
/// path-planning main structure
struct PathPlanning
{
	void AStar(Map_Element rob_pos, Map_Element destination);
	void add_root_in_tree(std::shared_ptr<TreeNode> nodeA);
	void add_destination_in_tree(std::shared_ptr<TreeNode> nodeB);
	void find_shortest_path();
	void update_fringe(std::pair<double, std::shared_ptr<TreeNode>> next);
	std::pair<double, std::shared_ptr<TreeNode>> find_smallest_in_fringe();
	void update_weights(std::pair<double, std::shared_ptr<TreeNode>> next);
	void init_tree();
	bool isConnectable( Map_Element, Map_Element);

	std::unique_ptr<std::vector< std::vector<double>>> obstacles_coordinates_;
	std::unique_ptr<std::vector<std::shared_ptr<TreeNode>>> vertices_;
	std::unique_ptr<std::vector<Obstacle>> obstacles_;
	std::unique_ptr<PathTree> tree_;
	std::unique_ptr<std::list<Map_Element>> shortest_path;
	std::list<std::unique_ptr<std::pair<double, std::shared_ptr<TreeNode>>>> fringe_;
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
