/*! 
 * \author Group 2
 * \file path_regulation_gr2.h
 * \brief regulation to follow a given path
 */

#ifndef _PATH_REGULATION_GR2_H_
#define _PATH_REGULATION_GR2_H_

#include "CtrlStruct_gr2.h"

// Constantes
#define COORDS			2
#define I				0
#define J				1
#define GOAL			2

NAMESPACE_INIT(ctrlGr2);

void follow_path(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
