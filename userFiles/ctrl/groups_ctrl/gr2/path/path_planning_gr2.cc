#include "path_planning_gr2.h"
#include "init_pos_gr2.h"
#include "opp_pos_gr2.h"
#include "useful_gr2.h"
#include <math.h>

NAMESPACE_INIT(ctrlGr2);

static double centre_obstacle[][] = { 
	0.04,	0.16,
	0.34,	0.16,
	0.34,	0.54,
	-0.34,	0.54,
	-0.34,	0.24,
	-0.64,	0.24,
	-0.64,	-0.24,
	-0.34,	-0.24,
	-0.34,	-0.54,
	0.34,	-0.54,
	0.34,	-0.16,
	0.04,	-0.16
}

static double wall_1[][] = {
	0.64,	1.36,
	0.64,	0.86,
	0.34,	0.86,
	0.34,	1.36,
}

static double wall_2[][] = {
	-0.86,	0.99,
	-0.36,	0.99,
	-0.36,	0.69,
	-0.86,	0.69,
}

static double wall_3[][] = {
	-0.86, -0.69,
	-0.36, -0.69,
	-0.36, -0.99,
	-0.86, -0.99,
}

static_double wall_4[][] = {
	0.34, -1.36,
	0.34, -0.86,
	0.64, -0.86,
	0.64, -1.36,

}

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
