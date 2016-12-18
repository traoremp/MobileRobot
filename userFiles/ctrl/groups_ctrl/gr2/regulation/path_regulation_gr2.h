/*! 
 * \author Group 2
 * \file path_regulation_gr2.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR2_H_
#define _PATH_REGULATION_GR2_H_

#include "CtrlStruct_gr2.h"

// Constantes
#define COORDS				2
#define X					0
#define Y					1


#define DIST_THRESHOLD		110 	//threshold for repulsive field
#define K_ATT 				0.02
#define	K_REP				260000
#define K_OPP				3000000

#define ROT_SPEED 			1.8 //constante used for forcetocommand to specify the rotation speed 
#define OB_H				0
#define OB_L				1
#define OB_X				2
#define OB_Y				3

NAMESPACE_INIT(ctrlGr2);

void follow_path(CtrlStruct *cvs);
void ForceToCommand(double F[], CtrlStruct *cvs, double turn);

NAMESPACE_CLOSE();

#endif
