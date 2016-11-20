#include "path_planning_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "useful_gr2.h"
#include <math.h>
#include <iostream>

NAMESPACE_INIT(ctrlGr2);

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
	
	//array of the map with position of elements we need (obstacles, goal,...)
	int map[MAP_LENGTH][MAP_WIDTH];
	
	//position of the goal
	//int goal_i = 15;
	//int goal_j = 30;
	
	//variables used for potential field algorithm
	//float dt = 0.1;
	
	//int distanceThreshold=20; //threshold for repulsive field
	//float k_att = 0.1;
	//int k_rep = 600;
	//int F_att_max = 30;
	//int F_att_min = 12;
	//int F_rep_max = 50;

	//float rot_speed = 1.8; %constante utilisé dans ForceToCommande,spécifie à quelle vitesse le robot tourne

	//int v_max = 30; %vitesse angulaire max des roues
	
	int i;
	int j;

	// ---initialisation of the array map---
	for(i = 0; i<MAP_LENGTH; i++)
		for(j = 0; j<MAP_WIDTH; j++)
			map[i][j] = 0;
		
	// ----------limits of the map-----------
	//wall of the top
	for(j = 0; j<MAP_WIDTH; j++)
		map[0][j] = 1;
	
	//wall of the right
	for(i = 0; i<MAP_LENGTH; i++)
		map[i][MAP_WIDTH-1] = 1;
	
	//wall of the bottom
	for(j = 0; j<MAP_WIDTH; j++)
		map[MAP_LENGTH-1][j] = 1;
	
	//wall of the left
	for(i = 0; i<MAP_LENGTH; i++)
		map[i][0] = 1;

	// --little walls of the starting blocks--
	
	//Top left wall
	for(j = 0; j<TL_WALL_j; j++)
		map[TL_WALL_i][j] = 1;
	
	//Bot left wall
	for(j = 0; j<TL_WALL_j; j++)
		map[BL_WALL_i][j] = 1;
	
	//Top right wall
	for(i = 0; i<TR_WALL_i; i++)
		map[i][TR_WALL_j] = 1;
	
	//Bot right wall
	for(i = BR_WALL_i; i<MAP_LENGTH; i++)
		map[i][BR_WALL_j] = 1;
	
	// --obstacle of the center--
	
	//top segment
	for(j = TL_SEG_j; j<TR_SEG_j; j++)
		for(i = TL_SEG_i; i<TL_SEG_i + SEG_WIDTH; i++)
			map[i][j] = 1;
	
	//bot segment
	for(j = TL_SEG_j; j<TR_SEG_j; j++)
		for(i = BL_SEG_i; i<BL_SEG_i + SEG_WIDTH; i++)
			map[i][j] = 1;
	
	//center segment
	for(j = CL_SEG_j; j<CR_SEG_j; j++)
		for(i = CL_SEG_i; i<CL_SEG_i + 2*SEG_WIDTH; i++)
			map[i][j] = 1;
		
	//connection between three segements
	for(i = TL_SEG_i; i<BL_SEG_i; i++)
		for(j = TL_SEG_j; j<TL_SEG_j + SEG_WIDTH; j++)
			map[i][j] = 1;
	
	//map[goal_i][goal_j] = GOAL;
	
	//affichage
	for(i = 0; i<MAP_LENGTH; i++)
	{
		for(j = 0; j<MAP_WIDTH; j++)
			std::cout << map[i][j] << ",";
	std::cout << std::endl;
	}
	
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

NAMESPACE_CLOSE();
