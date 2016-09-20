/*! 
 * \author Nicolas Van der Noot
 * \file user_shapes.hh
 * \brief file used by the user to specify the shapes used
 */

#ifndef _USER_SHAPES_HH_
#define _USER_SHAPES_HH_

#include "MainUnionShape.hh"
#include "mbs_data.h"

namespace ContactGeom{

// function prototype
void config_shapes(MainUnionShape *main_union, MbsData *mbs_data);

}
#endif
