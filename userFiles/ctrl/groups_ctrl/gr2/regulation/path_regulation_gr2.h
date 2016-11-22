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
#define I					0
#define J					1
#define GOAL				2


#define DIST_THRESHOLD		20 	//threshold for repulsive field
#define K_ATT 				1
#define	K_REP				60
#define	F_ATT_MAX			40
#define	F_ATT_MIN			10
#define	F_REP_MAX			50
#define ROT_SPEED 			1.8 //constante used for forcetocommand to specify the rotation speed 
#define V_MAX				30 //vitesse angulaire max des roues

NAMESPACE_INIT(ctrlGr2);

void follow_path(CtrlStruct *cvs);
void ForceToCommand(float F[], CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
