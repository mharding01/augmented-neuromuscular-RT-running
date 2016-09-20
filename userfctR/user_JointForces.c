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
#include "user_IO.h"
#include "SimuOptions.h"
#include "actuators_interface.h"

double* user_JointForces(MbsData *mbs_data, double tsim)
{
	compute_actuators_torque(mbs_data, mbs_data->ux, mbs_data->uxd, mbs_data->Qq);

	if ((mbs_data->user_IO->options->coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (mbs_data->user_IO->options->coman_model == SPRING_TOE_FEET_COMAN))
	{
		compute_spring_toe(mbs_data);
	}

	return mbs_data->Qq;
}
