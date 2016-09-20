//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "MBSdef.h"
#include "mbs_data.h"
#include "actuators_interface.h"

void user_Derivative(MbsData *mbs_data)
{
	compute_actuators_derivative(mbs_data, mbs_data->ux, mbs_data->uxd);
}
