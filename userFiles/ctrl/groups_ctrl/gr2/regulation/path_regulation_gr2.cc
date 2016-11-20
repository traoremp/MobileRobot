#include "path_regulation_gr2.h"
#include "path_planning_gr2.h"
#include "useful_gr2.h"
#include "speed_regulation_gr2.h"

NAMESPACE_INIT(ctrlGr2);

/*! \brief follow a given path
 * 
 * \param[in,out] cvs controller main structure
 */
void follow_path(CtrlStruct *cvs)
{
	PathPlanning* path = cvs->path;
}

NAMESPACE_CLOSE();
