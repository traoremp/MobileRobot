/*! 
 * \author Group 2
 * \file mcl_gr2.h
 * \brief localization using Monte Carlo technic
 */

#ifndef _MCL_GR2_H_
#define _MCL_GR2_H_

#include "CtrlStruct_gr2.h"
#include "init_pos_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/// Markov main structure
struct MCLStruct
{
	int dummy_variable; ///< put your own variable, this is just an example without purpose
};

void mcl_update(CtrlStruct *cvs);

NAMESPACE_CLOSE();

#endif
