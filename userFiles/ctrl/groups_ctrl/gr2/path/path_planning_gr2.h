/*! 
 * \author Group 2
 * \file path_planning_gr2.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR2_H_
#define _PATH_PLANNING_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"

//number of cells (length and width) of the map (2000x3000 mm) so their size are 50x50 mm
#define MAP_LENGTH		60
#define MAP_WIDTH		40
#define CELL_SIZE		50
#define SEG_WIDTH		100/CELL_SIZE //width of segments
#define COORDS			2

#define ROBOT_SIZE		130.0/CELL_SIZE

//end position i,j of top left, bot left, top right, bot right little walls
#define TL_WALL_i		650/CELL_SIZE  
#define TL_WALL_j		500/CELL_SIZE 	

#define BL_WALL_i		(2350-CELL_SIZE)/CELL_SIZE
#define BL_WALL_j		500/CELL_SIZE

#define TR_WALL_i		500/CELL_SIZE
#define TR_WALL_j		1500/CELL_SIZE

#define BR_WALL_i		2500/CELL_SIZE
#define BR_WALL_j		1500/CELL_SIZE

//segment position i,j of the center obstacle

//top segment
#define TL_SEG_i		(1100-CELL_SIZE)/CELL_SIZE
#define TL_SEG_j		800/CELL_SIZE

#define TR_SEG_i		(1100-CELL_SIZE)/CELL_SIZE
#define TR_SEG_j		1200/CELL_SIZE

//bot segement
#define BL_SEG_i		(1900-CELL_SIZE)/CELL_SIZE
#define BL_SEG_j		800/CELL_SIZE

#define BR_SEG_i		(1900-CELL_SIZE)/CELL_SIZE
#define BR_SEG_j		1200/CELL_SIZE

//center segment
#define CL_SEG_i		(1400-CELL_SIZE)/CELL_SIZE
#define CL_SEG_j		500/CELL_SIZE

#define CR_SEG_i		(1400-CELL_SIZE)/CELL_SIZE
#define CR_SEG_j		800/CELL_SIZE


NAMESPACE_INIT(ctrlGr2);

/// path-planning main structure
struct PathPlanning
{
	int map[MAP_LENGTH][MAP_WIDTH]; //array of the map with position of elements we need (obstacles, goal,...)
	int goal_pos[COORDS]; // position of the goal
	float last_t;
	bool wait;
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
