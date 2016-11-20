#include "path_planning_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "useful_gr2.h"
#include <math.h>
#include <algorithm>

NAMESPACE_INIT(ctrlGr2);

static const double centre_obstacle[] = {
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

static const double wall_1[] = {
	0.64,	1.36,
	0.64,	0.86,
	0.34,	0.86,
	0.34,	1.36,
};

static const double wall_2[] = {
	-0.86,	0.99,
	-0.36,	0.99,
	-0.36,	0.69,
	-0.86,	0.69,
};

static const double wall_3[] = {
	-0.86, -0.69,
	-0.36, -0.69,
	-0.36, -0.99,
	-0.86, -0.99,
};

static const double wall_4[] = {
	0.34, -1.36,
	0.34, -0.86,
	0.64, -0.86,
	0.64, -1.36,

};
Obstacle::Obstacle(std::list<std::shared_ptr<Map_Element>> points){
	points_ = std::move(points);
}
void TreeNode::add_child(std::unique_ptr<std::pair<int, std::shared_ptr<TreeNode>>> vertex) {
	children_.push_back(std::move(vertex));
}

/*! \brief initialize the path-planning algorithm (memory allocated)
 * 
 * \param[in,out] path path-planning main structure
 */
PathPlanning* init_path_planning()
{
	PathPlanning *path;

	// memory allocation
	path = (PathPlanning*) malloc(sizeof(PathPlanning));

	// ----- path-planning initialization start ----- //
	std::vector<const double*> obstacles_coordinates(0);
	obstacles_coordinates.push_back(centre_obstacle);
	obstacles_coordinates.push_back(wall_1);
	obstacles_coordinates.push_back(wall_2);
	obstacles_coordinates.push_back(wall_3);
	obstacles_coordinates.push_back(wall_4);
	// ----- path-planning initialization end ----- //
	std::for_each(obstacles_coordinates.begin(), obstacles_coordinates.end(), [&](double* tab){
		int i = 1;
		Obstacle obs;
		Map_Element point;
		while (tab[i]) {
			point << tab[i - 1], tab[i];
			path->vertices_->push_back(std::make_shared<TreeNode>(point));
			obs.add_point(point);
			i+=2;
		}
		path->obstacles_->push_back(obs);
	});
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
void PathPlanning::init_tree(Map_Element rob_pos, Map_Element destination) {
	std::shared_ptr<TreeNode> root = std::make_shared<TreeNode>(rob_pos);
	std::shared_ptr<TreeNode> destination_node = std::make_shared<TreeNode>(destination);
	vertices_->push_back(root);
	vertices_->push_back(std::make_shared<TreeNode>(Map_Element(-0.91, 1.41)));
	vertices_->push_back(std::make_shared<TreeNode>(Map_Element(0.91, 1.41)));
	vertices_->push_back(std::make_shared<TreeNode>(Map_Element(-0.91, -1.41 )));
	vertices_->push_back(std::make_shared<TreeNode>(Map_Element(0.91, -1.41)));
	vertices_->push_back(std::make_shared<TreeNode>(destination));
	std::for_each(vertices_->begin(), vertices_->end(), [&](std::shared_ptr<TreeNode> nodeA) {
		std::for_each(vertices_->begin(), vertices_->end(), [&](std::shared_ptr<TreeNode> nodeB) {
			Map_Element pointA = nodeA->getPosition();
			Map_Element pointB = nodeB->getPosition();
			if (PathPlanning::isConnectable(pointA, pointB)) {
				std::unique_ptr<std::pair<int, std::shared_ptr<TreeNode>>> node = std::make_unique<std::pair<int, std::shared_ptr<TreeNode>>>();
				node->first = (pointB - pointA).norm();
				node->second = nodeB;
				nodeA->add_child(std::move(node));
			}
		});
	});
	PathPlanning::tree_ = std::make_unique<PathTree>(root, destination_node);

}
bool PathPlanning::isConnectable( Map_Element OA, Map_Element OB)
{
	double angle;
	const int k = 400; // subdiviser le segment reliant A et B et tester 400points differents de la liaison pour detecter une eventuelle collision
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
			while( it != points.end()) {
				previous_it = it;
				(it++);
				Map_Element vec_1 = *(previous_it)->get() - point_to_test;
				Map_Element vec_2 = *(it)->get() - point_to_test;
				angle += std::acos((vec_1.dot(vec_2))/(vec_1.norm() * vec_2.norm()));
				
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

