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

	int i;
	int j;

	path->wait = 0;//OFF

	// ---initialisation of the array map---
	for(i = 0; i<MAP_LENGTH; i++)
		for(j = 0; j<MAP_WIDTH; j++)
			path->map[i][j] = 0;
		
	// ----------limits of the map-----------
	//wall of the top
	for(j = 0; j<MAP_WIDTH; j++)
		path->map[0][j] = 1;
	
	//wall of the right
	for(i = 0; i<MAP_LENGTH; i++)
		path->map[i][MAP_WIDTH-1] = 1;
	
	//wall of the bottom
	for(j = 0; j<MAP_WIDTH; j++)
		path->map[MAP_LENGTH-1][j] = 1;
	
	//wall of the left
	for(i = 0; i<MAP_LENGTH; i++)
		path->map[i][0] = 1;

	// --little walls of the starting blocks--
	
	//Top left wall
	for(j = 0; j<TL_WALL_j; j++)
		path->map[TL_WALL_i][j] = 1;
	
	//Bot left wall
	for(j = 0; j<TL_WALL_j; j++)
		path->map[BL_WALL_i][j] = 1;
	
	//Top right wall
	for(i = 0; i<TR_WALL_i; i++)
		path->map[i][TR_WALL_j] = 1;
	
	//Bot right wall
	for(i = BR_WALL_i; i<MAP_LENGTH; i++)
		path->map[i][BR_WALL_j] = 1;
	
	// --------obstacle of the center---------
	
	//top segment
	for(j = TL_SEG_j; j<TR_SEG_j; j++)
		for(i = TL_SEG_i; i<TL_SEG_i + SEG_WIDTH; i++)
			path->map[i][j] = 1;
	
	//bot segment
	for(j = TL_SEG_j; j<TR_SEG_j; j++)
		for(i = BL_SEG_i; i<BL_SEG_i + SEG_WIDTH; i++)
			path->map[i][j] = 1;
	
	//center segment
	for(j = CL_SEG_j; j<CR_SEG_j; j++)
		for(i = CL_SEG_i; i<CL_SEG_i + 2*SEG_WIDTH; i++)
			path->map[i][j] = 1;
		
	//connection between three segements
	for(i = TL_SEG_i; i<BL_SEG_i; i++)
		for(j = TL_SEG_j; j<TL_SEG_j + SEG_WIDTH; j++)
			path->map[i][j] = 1;
	
	for (i = TR_SEG_i; i<BR_SEG_i; i++)
			path->map[i][TR_SEG_j] = 1;

	//affichage
	for(i = 0; i<MAP_LENGTH; i++)
	{
		for(j = 0; j<MAP_WIDTH; j++)
			std::cout << path->map[i][j] << ",";
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
