/*! 
 * \author Nicolas Van der Noot
 * \file user_contact_states.hh
 * \brief user own states for contact model
 */

#ifndef _USER_CONTACT_STATES_HH_
#define _USER_CONTACT_STATES_HH_

#include "mbs_data.h"
#include "BasicShape.hh"

namespace ContactGeom{

// fucntion prototype
void user_contact_states(BasicShape *shape, MbsData *mbs_data);

}
#endif
