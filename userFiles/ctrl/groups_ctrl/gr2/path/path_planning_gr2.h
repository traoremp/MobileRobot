/*! 
 * \author Group 2
 * \file path_planning_gr2.h
 * \brief path-planning algorithm
 */

#ifndef _PATH_PLANNING_GR2_H_
#define _PATH_PLANNING_GR2_H_ 
 
#include "namespace_ctrl.h"
#include "CtrlStruct_gr2.h"

//number of rectangles in the map (2000x3000 mm) and their four specifications (width,length, coord x,y of center)
#define NB_RECT			24
#define SPEC			4
#define COORDS			2
#define WIDTH			20
#define ROBOT_SIZE		130

// --- Rectangles of map boundary --- //

//top wall
#define BT_WIDTH		20					//width of rectangle 
#define BT_LENGTH		2000				//length of rectangle
#define BT_CENTER_X		0					//x coord of center
#define BT_CENTER_Y		1500+WIDTH/2		//y coord of center

//right wall
#define BR_WIDTH		3000
#define BR_LENGTH		20
#define BR_CENTER_X		1000+WIDTH/2
#define BR_CENTER_Y		0

//boT wall
#define BB_WIDTH		20
#define BB_LENGTH		2000
#define BB_CENTER_X		0
#define BB_CENTER_Y		-1500-WIDTH/2

//left wall
#define BL_WIDTH		3000
#define BL_LENGTH		20
#define BL_CENTER_X		-1000-WIDTH/2
#define BL_CENTER_Y		0

// --- Rectangles of horizontal walls --- //

//top wall
#define HT_WIDTH		20
#define HT_LENGTH		500
#define HT_CENTER_X		-500-HT_LENGTH/2
#define HT_CENTER_Y		840

//bot wall
#define HB_WIDTH		20
#define HB_LENGTH		500
#define HB_CENTER_X		-500-HB_LENGTH/2
#define HB_CENTER_Y		-840

// --- Rectangles of vertical walls --- //

//top wall
#define VT_WIDTH		500
#define VT_LENGTH		20
#define VT_CENTER_X		490
#define VT_CENTER_Y		1250

//bot wall
#define VB_WIDTH		500
#define VB_LENGTH		20
#define VB_CENTER_X		490
#define VB_CENTER_Y		-1250

// --- Four rectangles of center obstacle --- //

//top
#define CT_WIDTH		100
#define CT_LENGTH		400
#define CT_CENTER_X		0
#define CT_CENTER_Y		350

//bot
#define CB_WIDTH		100
#define CB_LENGTH		400
#define CB_CENTER_X		0
#define CB_CENTER_Y		-350

//betwin
#define CBE_WIDTH		600
#define CBE_LENGTH		100
#define CBE_CENTER_X	-150
#define CBE_CENTER_Y	0

//center
#define CC_WIDTH		200
#define CC_LENGTH		300
#define CC_CENTER_X		-350
#define CC_CENTER_Y		0

// --- Virtual wall --- //

#define CV1_WIDTH		200
#define CV1_LENGTH		200
#define CV1_CENTER_X	-300
#define CV1_CENTER_Y	200

#define CV2_WIDTH		200
#define CV2_LENGTH		200
#define CV2_CENTER_X	-300
#define CV2_CENTER_Y	-200

#define CV3_WIDTH		600
#define CV3_LENGTH		20
#define CV3_CENTER_X	190
#define CV3_CENTER_Y	0

#define BLV1_WIDTH		400
#define BLV1_LENGTH		100
#define BLV1_CENTER_X	-950
#define BLV1_CENTER_Y	-600

#define BLV2_WIDTH		300
#define BLV2_LENGTH		100
#define BLV2_CENTER_X	-850
#define BLV2_CENTER_Y	-650

#define BLV3_WIDTH		200
#define BLV3_LENGTH		100
#define BLV3_CENTER_X	-750
#define BLV3_CENTER_Y	-700

#define BLV4_WIDTH		100
#define BLV4_LENGTH		100
#define BLV4_CENTER_X	-650
#define BLV4_CENTER_Y	-750


#define TLV1_WIDTH		400
#define TLV1_LENGTH		100
#define TLV1_CENTER_X	-950
#define TLV1_CENTER_Y	600

#define TLV2_WIDTH		300
#define TLV2_LENGTH		100
#define TLV2_CENTER_X	-850
#define TLV2_CENTER_Y	650

#define TLV3_WIDTH		200
#define TLV3_LENGTH		100
#define TLV3_CENTER_X	-750
#define TLV3_CENTER_Y	700

#define TLV4_WIDTH		100
#define TLV4_LENGTH		100
#define TLV4_CENTER_X	-650
#define TLV4_CENTER_Y	750

// --- OPPONENT --- //

#define OPP_WIDTH		350
#define OPP_LENGTH		350
#define OPP_CENTER_X	1200
#define OPP_CENTER_Y	0



NAMESPACE_INIT(ctrlGr2);

/// path-planning main structure
struct PathPlanning
{
	//array of the map with position of rectangles which forms obstacles
	int map[NB_RECT][SPEC] = { { BT_WIDTH ,BT_LENGTH ,BT_CENTER_X ,BT_CENTER_Y },{ BR_WIDTH ,BR_LENGTH ,BR_CENTER_X ,BR_CENTER_Y },
								{ BB_WIDTH ,BB_LENGTH ,BB_CENTER_X ,BB_CENTER_Y },{ BL_WIDTH ,BL_LENGTH ,BL_CENTER_X ,BL_CENTER_Y },
								{ HT_WIDTH , HT_LENGTH ,HT_CENTER_X ,HT_CENTER_Y },{ HB_WIDTH ,HB_LENGTH ,HB_CENTER_X ,HB_CENTER_Y },
								{ VT_WIDTH ,VT_LENGTH ,VT_CENTER_X ,VT_CENTER_Y },{ VB_WIDTH ,VB_LENGTH ,VB_CENTER_X ,VB_CENTER_Y },
								{ CT_WIDTH ,CT_LENGTH ,CT_CENTER_X ,CT_CENTER_Y },{ CB_WIDTH ,CB_LENGTH ,CB_CENTER_X ,CB_CENTER_Y },
								{ CBE_WIDTH ,CBE_LENGTH ,CBE_CENTER_X ,CBE_CENTER_Y },{ CC_WIDTH ,CC_LENGTH ,CC_CENTER_X ,CC_CENTER_Y },
								{ CV1_WIDTH ,CV1_LENGTH ,CV1_CENTER_X ,CV1_CENTER_Y },{ CV2_WIDTH ,CV2_LENGTH ,CV2_CENTER_X ,CV2_CENTER_Y },
								{ BLV1_WIDTH ,BLV1_LENGTH ,BLV1_CENTER_X ,BLV1_CENTER_Y },{ BLV2_WIDTH ,BLV2_LENGTH ,BLV2_CENTER_X ,BLV2_CENTER_Y },
								{ BLV3_WIDTH ,BLV3_LENGTH ,BLV3_CENTER_X ,BLV3_CENTER_Y },{ BLV4_WIDTH ,BLV4_LENGTH ,BLV4_CENTER_X ,BLV4_CENTER_Y },
								{ TLV1_WIDTH ,TLV1_LENGTH ,TLV1_CENTER_X ,TLV1_CENTER_Y },{ TLV2_WIDTH ,TLV2_LENGTH ,TLV2_CENTER_X ,TLV2_CENTER_Y },
								{ TLV3_WIDTH ,TLV3_LENGTH ,TLV3_CENTER_X ,TLV3_CENTER_Y },{ TLV4_WIDTH ,TLV4_LENGTH ,TLV4_CENTER_X ,TLV4_CENTER_Y },
								{ OPP_WIDTH ,OPP_LENGTH ,OPP_CENTER_X ,OPP_CENTER_Y },{ CV3_WIDTH ,CV3_LENGTH ,CV3_CENTER_X ,CV3_CENTER_Y } };
	
	int goal_pos[COORDS]; // position of the goal
	float last_t;
	bool wait;
	int cage_open = 0; //initialise closed
};

PathPlanning* init_path_planning();
void free_path_planning(PathPlanning *path);

NAMESPACE_CLOSE();

#endif
