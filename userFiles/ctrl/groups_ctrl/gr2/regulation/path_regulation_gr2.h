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


#define DIST_THRESHOLD		200 	//threshold for repulsive field
#define K_ATT 				0.02
#define	K_REP				200
//#define	F_ATT_MAX			40
//#define	F_ATT_MIN			10
//#define	F_REP_MAX			60
#define ROT_SPEED 			2.5 //constante used for forcetocommand to specify the rotation speed 
#define OB_H	0
#define OB_L	1
#define OB_X	2
#define OB_Y	3


NAMESPACE_INIT(ctrlGr2);

void follow_path(CtrlStruct *cvs);
void ForceToCommand(float F[], CtrlStruct *cvs);
void limitNorm(float F[], int maxNorm, int minNorm);

NAMESPACE_CLOSE();

#endif
