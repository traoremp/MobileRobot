#include "path_planning_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "useful_gr2.h"
#include <math.h>
#include <algorithm>
#include <limits>
#include <intrin.h>
NAMESPACE_INIT(ctrlGr2);


Obstacle::Obstacle(std::list<std::shared_ptr<Map_Element>> points){
	points_ = std::move(points);
}
void TreeNode::add_child(std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>> vertex) {
	children_.emplace_back(vertex);
}

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */
PathPlanning* init_path_planning()
{
	PathPlanning *path;

	// memory allocation
	path = new PathPlanning();
	std::vector<double> centre_obstacle_1 = {
		-0.64, 0.24,
		0.04, 0.24,
		0.04, -0.24,
		-0.64, -0.24,
	};
	std::vector<double> centre_obstacle_2 = {
		-0.34, -0.54,
		-0.34, 0.54,
		0.04, 0.54,
		0.04, -0.54,
	};
	std::vector<double> centre_obstacle_3 = {
		-0.34, -0.54,
		-0.34, -0.16,
		0.34, -0.16,
		0.34, -0.54,
	};
	std::vector<double> centre_obstacle_4 = {
		-0.34, 0.16,
		-0.34, 0.54,
		0.34, 0.54,
		0.34, 0.16,
	};

	std::vector<double> wall_1 = std::vector<double>{
		0.64,	1.562,
		0.64,	0.86,
		0.34,	0.86,
		0.34,	1.562,
	};
	
	std::vector<double> wall_2 = std::vector<double>{
		-1.062,	0.99,
		-0.36,	0.99,
		-0.36,	0.69,
		-1.062,	0.69,
	};

	std::vector<double> wall_3 = std::vector<double>{
		-1.062, -0.69,
		-0.36, -0.69,
		-0.36, -0.99,
		-1.062, -0.99,
	};

	std::vector<double> wall_4 = std::vector<double>{
		0.34, -1.562,
		0.34, -0.86,
		0.64, -0.86,
		0.64, -1.562,

	};
	// ----- path-planning initialization start ----- //
	path->obstacles_coordinates_ = std::make_unique<std::vector<std::vector<double>>>();
	path->obstacles_coordinates_->emplace_back(centre_obstacle_1);
	path->obstacles_coordinates_->emplace_back(centre_obstacle_2);
	path->obstacles_coordinates_->emplace_back(centre_obstacle_3);
	path->obstacles_coordinates_->emplace_back(centre_obstacle_4);
	path->obstacles_coordinates_->emplace_back(wall_1);
	path->obstacles_coordinates_->emplace_back(wall_2);
	path->obstacles_coordinates_->emplace_back(wall_3);
	path->obstacles_coordinates_->emplace_back(wall_4);
	
	// ----- path-planning initialization end ----- //
	
	// return structure initialized
	return path;
}

/*! \brief close the path-planning algorithm (memory released)
 * 
 * \param[in,out] path path-planning main structure
 */
void free_path_planning(PathPlanning *path)
{
	// ----- path-planning memory release start ----- //
	

	// ----- path-planning memory release end ----- //

	free(path);
}
void PathPlanning::AStar()
{
	auto root = tree_->getRoot();
	auto destination = tree_->getTarget();
	fringe_.clear();
	auto root_fringe = std::pair<double, std::shared_ptr<TreeNode>>(0.0, root);
	auto destination_fringe = std::pair<double, std::shared_ptr<TreeNode>>(std::numeric_limits<double>::infinity(), destination);
	fringe_.emplace_back(std::make_unique<std::pair<double, std::shared_ptr<TreeNode>>>(root_fringe));
	find_shortest_path();

}
void PathPlanning::find_shortest_path() {
	std::pair<double, std::shared_ptr<TreeNode>> next;
	shortest_path = std::make_unique<std::list<Map_Element>>();
	while (next.second != tree_->getTarget()) {
		next = find_smallest_in_fringe();
		update_weights(next.second);
		update_fringe(next);
		shortest_path->emplace_back(next.second->getPosition());

	}
	return;
}
void PathPlanning::update_fringe(std::pair<double, std::shared_ptr<TreeNode>> next) {
	std::shared_ptr<TreeNode> next_node = next.second;
	for (auto& it_next_node = next_node->getChildren().begin(); it_next_node != next_node->getChildren().end();it_next_node++){
		for (auto& it_fringe = fringe_.begin(); it_fringe != fringe_.end(); it_fringe++){
			if ((*it_next_node)->second == (*it_fringe)->second) {
				(*it_fringe)->first = next.first + (*it_next_node)->first;
			}
		}
	};
}
std::pair<double, std::shared_ptr<TreeNode>> PathPlanning::find_smallest_in_fringe() {
	std::pair<double, std::shared_ptr<TreeNode>> smallest_distance;
	double distance = std::numeric_limits<double>::max();
	std::list<std::unique_ptr<std::pair<double, std::shared_ptr<TreeNode>>>>::iterator it_to_smallest = std::list<std::unique_ptr<std::pair<double, std::shared_ptr<TreeNode>>>>::iterator();
	for (auto it = fringe_.begin();  it != fringe_.end(); it++)
	{		
		if ((*it)->first < distance) {
			smallest_distance = (**it);
			it_to_smallest = it;
			distance = (*it)->first;
		}

	};
	return smallest_distance;
}
void PathPlanning::update_weights(std::shared_ptr<TreeNode> node) {
	fringe_.clear();
	for(auto& it = node->getChildren().begin(); it != node->getChildren().end(); it++){	
		(*it)->first = (*it)->first + (tree_->getTarget()->getPosition() - (*it)->second->getPosition()).norm() - (tree_->getTarget()->getPosition() - node->getPosition()).norm();
		auto newNodeFringe = std::pair<double, std::shared_ptr<TreeNode>>(std::numeric_limits<double>::infinity(), (*it)->second);
		fringe_.emplace_back(std::make_unique<std::pair<double, std::shared_ptr<TreeNode>>>(newNodeFringe));
	}

}

void PathPlanning::init_tree(Map_Element rob_pos, Map_Element destination) {
	
	vertices_ = std::make_unique<std::vector<std::shared_ptr<TreeNode>>>();
	obstacles_ = std::make_unique<std::vector<Obstacle>>();
	std::for_each(obstacles_coordinates_->begin(), obstacles_coordinates_->end(), [&](std::vector<double> tab) {
		int i = 1;
		Obstacle obs;
		Map_Element point;
		while (i <= tab.size()) {
			point << tab[i - 1], tab[i];
			vertices_->emplace_back(std::make_shared<TreeNode>(point));
			obs.add_point(point);
			i += 2;
		}
		obstacles_->emplace_back(obs);
	});

	std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>(rob_pos);
	std::shared_ptr<TreeNode> destination_node = std::make_shared<TreeNode>(destination);
	vertices_->emplace_back(root);
	vertices_->emplace_back(destination_node);
	std::for_each(vertices_->begin(), vertices_->end(), [&](std::shared_ptr<TreeNode> nodeA) {
		if (nodeA->getPosition() == destination)
			return;
		std::for_each(vertices_->begin(), vertices_->end(), [&](std::shared_ptr<TreeNode> nodeB) {			
			Map_Element pointA = nodeA->getPosition();
			Map_Element pointB = nodeB->getPosition();
			if ( pointB == rob_pos)
				return;
			if (PathPlanning::isConnectable(pointA, pointB)) {
				std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>> node = std::make_shared<std::pair<double, std::shared_ptr<TreeNode>>>();
				node->first = (pointB - pointA).norm();
				node->second = nodeB;
				nodeA->add_child(node);
			}
			
			
		});
	});
	PathPlanning::tree_ = std::make_unique<PathTree>(root, destination_node);

}
bool PathPlanning::isConnectable( Map_Element OA, Map_Element OB)
{
	if(OA[0] < -1.0 || OB[0]  < -1.0 || OA[1] < -1.5 || OB[1] < -1.5 || OA[1]  > 1.5 || OB[1]  > 1.5)
		return false;
	double angle;
	const int k = 50; // subdiviser le segment reliant A et B et tester 400points differents de la liaison pour detecter une eventuelle collision
	Map_Element vec_AB = OB.matrix() - OA.matrix();
	if (vec_AB.norm() == 0.0)
		return false;
	vec_AB /= k;
	for (int i = 1; i <= k; i++) {
		Map_Element point_to_test = (OA.matrix() + i * vec_AB.matrix());
		for (auto& obstacle : *this->obstacles_) {
			angle = 0;
			std::list<std::shared_ptr<Map_Element>>& points = obstacle.getPoints();
			std::list<std::shared_ptr<Map_Element>>::iterator it = points.begin();
			std::list<std::shared_ptr<Map_Element>>::iterator previous_it = it;
			while( ++it != points.end()) {				
				Map_Element vec_1 = (*previous_it)->matrix() - point_to_test.matrix();
				vec_1.normalize();
				Map_Element vec_2 = (*it)->matrix() - point_to_test.matrix();
				vec_2.normalize();
				double angle_between = std::acos(vec_1.dot(vec_2));
				if((std::ceil(angle_between * 100) / 100 != std::ceil(M_PI * 100) / 100))
					angle += angle_between;
				previous_it = it;
				
			}
			it = points.begin();
			Map_Element vec_1 = (*previous_it)->matrix() - point_to_test.matrix();
			vec_1.normalize();
			Map_Element vec_2 = (*it)->matrix() - point_to_test.matrix();
			vec_2.normalize();
			double angle_between = std::acos(vec_1.dot(vec_2));
			if ((std::ceil(angle_between * 100) / 100 != std::ceil(M_PI * 100) / 100))
				angle += angle_between;
			if ((std::ceil(angle*100)/100 == std::ceil(2 * M_PI*100)/100))
				return false;	
		}

	}
	
	
	return true;
}
NAMESPACE_CLOSE();

