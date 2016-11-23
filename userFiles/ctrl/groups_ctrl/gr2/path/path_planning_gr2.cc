#include "path_planning_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "useful_gr2.h"
#include <math.h>
#include <algorithm>
#include <limits>

NAMESPACE_INIT(ctrlGr2);


Obstacle::Obstacle(std::list<std::shared_ptr<Map_Element>> points){
	points_ = std::move(points);
}
void TreeNode::add_child(std::shared_ptr<std::pair<double, std::shared_ptr<TreeNode>>> vertex) {
	children_.emplace_back(std::move(vertex));
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
	std::vector<double> centre_obstacle = std::vector<double>{
		0.04,	0.16,
		0.34,	0.16,
		0.34,	0.54,
		-0.34,	0.54,
		-0.34,	0.24,
		-0.64,	0.24,
		-0.64,	-0.24,
		-0.34,	-0.24,
		-0.34,	-0.54,
		0.34,	-0.54,
		0.34,	-0.16,
		0.04,	-0.16
	};

	std::vector<double> wall_1 = std::vector<double>{
		0.64,	1.36,
		0.64,	0.86,
		0.34,	0.86,
		0.34,	1.36,
	};

	std::vector<double> wall_2 = std::vector<double>{
		-0.86,	0.99,
		-0.36,	0.99,
		-0.36,	0.69,
		-0.86,	0.69,
	};

	std::vector<double> wall_3 = std::vector<double>{
		-0.86, -0.69,
		-0.36, -0.69,
		-0.36, -0.99,
		-0.86, -0.99,
	};

	std::vector<double> wall_4 = std::vector<double>{
		0.34, -1.36,
		0.34, -0.86,
		0.64, -0.86,
		0.64, -1.36,

	};
	// ----- path-planning initialization start ----- //
	path->obstacles_coordinates_ = std::make_unique<std::vector<std::vector<double>>>();
	path->obstacles_coordinates_->emplace_back(centre_obstacle);
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
	fringe_.emplace_back(std::make_unique<std::pair<double, std::shared_ptr<TreeNode>>>(destination_fringe));
	fill_fringe(root);//fonction recursive
	update_weights(root);//AStar
	find_shortest_path();

}
void PathPlanning::find_shortest_path() {
	std::pair<double, std::shared_ptr<TreeNode>> next;
	shortest_path = std::make_unique<std::list<Map_Element>>();
	while (!fringe_.empty() && next.second != tree_->getTarget()) {
		next = find_smallest_in_fringe();
		update_fringe(next);
		shortest_path->emplace_back(next.second->getPosition());

	}
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
	double distance = -1.0;
	std::list<std::unique_ptr<std::pair<double, std::shared_ptr<TreeNode>>>>::iterator& it_to_smallest = std::list<std::unique_ptr<std::pair<double, std::shared_ptr<TreeNode>>>>::iterator();
	for (auto& it = fringe_.begin();  it != fringe_.end(); it++)
	{
		
		if ((*it)->first < distance) {
			smallest_distance = (**it);
			it_to_smallest = it;
		}

	};
	fringe_.erase(it_to_smallest);
	return smallest_distance;
}
//void PathPlanning::update_weights(){
//	auto start = tree_->getRoot();
//	auto destination = tree_->getTarget();
//
//	/*std::shared_ptr<TreeNode> Astar_root = std::make_shared<TreeNode> (*start);
//	std::shared_ptr<TreeNode> Astar_destination = std::make_shared<TreeNode>(*destination);
//	PathTree Astar_path = PathTree(Astar_root, Astar_destination);*/
//	
//	update_weights(start);//recursive function
//}
void PathPlanning::update_weights(std::shared_ptr<TreeNode> node) {
	for(auto& it = node->getChildren().begin(); it != node->getChildren().end(); it++){
		//(*it)->first = (*it)->first + (tree_->getTarget()->getPosition() - (*it)->second->getPosition()).norm() - (tree_->getTarget()->getPosition() - node->getPosition()).norm();
		if ((*it)->second->getPosition() == tree_->getTarget()->getPosition() || (*it)->second->getPosition() == tree_->getRoot()->getPosition())
			return;
		update_weights((*it)->second);
	}
}
void PathPlanning::fill_fringe(std::shared_ptr<TreeNode> node){
	
	for(auto& it_child = node->getChildren().begin(); it_child != node->getChildren().end();it_child++){
		if ((*it_child)->second == tree_->getTarget())
			return;
		for (auto& it = fringe_.begin(); it != fringe_.end(); it++)
		{
			if ((*it)->second == (*it_child)->second) {
				return;
			}
		}
		auto newNodeFringe = std::pair<double, std::shared_ptr<TreeNode>>(std::numeric_limits<double>::infinity(), (*it_child)->second);
		fringe_.emplace_back(std::make_unique<std::pair<double, std::shared_ptr<TreeNode>>>(newNodeFringe));
		fill_fringe((*it_child)->second);
	};
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
	vertices_->emplace_back(std::make_shared<TreeNode>(Map_Element(-0.91, 1.41)));
	vertices_->emplace_back(std::make_shared<TreeNode>(Map_Element(0.91, 1.41)));
	vertices_->emplace_back(std::make_shared<TreeNode>(Map_Element(-0.91, -1.41 )));
	vertices_->emplace_back(std::make_shared<TreeNode>(Map_Element(0.91, -1.41)));
	vertices_->emplace_back(std::make_shared<TreeNode>(destination));
	std::for_each(vertices_->begin(), vertices_->end(), [&](std::shared_ptr<TreeNode> nodeA) {
		if (nodeA->getPosition() == destination)
			return;
		std::for_each(vertices_->begin(), vertices_->end(), [&](std::shared_ptr<TreeNode> nodeB) {			
			Map_Element pointA = nodeA->getPosition();
			Map_Element pointB = nodeB->getPosition();
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
	double angle;
	const int k = 20; // subdiviser le segment reliant A et B et tester 400points differents de la liaison pour detecter une eventuelle collision
	Map_Element vec_AB = OB - OA;
	if (vec_AB.norm() == 0)
		return false;
	vec_AB /= k;
	for (int i = 1; i <= k; i++) {
		Map_Element point_to_test = (OA + i * vec_AB);
		for (auto& obstacle : *this->obstacles_) {
			angle = 0;
			std::list<std::shared_ptr<Map_Element>>& points = obstacle.getPoints();
			std::list<std::shared_ptr<Map_Element>>::iterator& it = points.begin();
			std::list<std::shared_ptr<Map_Element>>::iterator& previous_it = it;
			while( ++it != points.end()) {				
				Map_Element vec_1 = *(previous_it)->get() - point_to_test;
				Map_Element vec_2 = *(it)->get() - point_to_test;
				angle += std::acos((vec_1.dot(vec_2))/(vec_1.norm() * vec_2.norm()));
				previous_it = it;
				
			}
			previous_it = it;
			it = points.begin();
			Map_Element vec_1 = *(previous_it)->get() - point_to_test;
			Map_Element vec_2 = *(it)->get() - point_to_test;
			angle += std::acos((vec_1.dot(vec_2)) / (vec_1.norm() * vec_2.norm()));
			if ((std::ceil(angle*100)/100 == std::ceil(2 * M_PI*100)/100))
				return false;	
		}

	}
	
	
	return true;
}
NAMESPACE_CLOSE();

