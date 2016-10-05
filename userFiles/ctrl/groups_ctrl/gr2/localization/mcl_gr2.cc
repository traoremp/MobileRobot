#include "mcl_gr2.h"
#include "odometry_gr2.h"
#include "triangulation_gr2.h"
#include "useful_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief update using MCL technic
 * 
 * \param[in,out] cvs controller main structure
 */
void mcl_update(CtrlStruct *cvs)
{
	// variable declaration
	RobotPosition *rob_pos;

	// variables initialization
	rob_pos = cvs->rob_pos;
}

NAMESPACE_CLOSE();
