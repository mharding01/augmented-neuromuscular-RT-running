
#include "HandleInputs.hh"
#include "ctrl_functions.hh"
#include "CtrlIndex.hh"

#include <stdio.h>

#define MAX_EXT_FORCE_Z_CONTROLLER 700.0 ///< limiting the forces sent to the controller, threshold [N]
#define THETA_HG_COR 0.0225              ///< trunk correction [rad]
#define SIZE_FEET_FORCES_VEC 50          ///< size of the feet vector [⁻]

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] fwd_kin forward kinematices
 */
HandleInputs::HandleInputs(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, ForwardKinematics *fwd_kin): Computation(inputs, options, ctrl_index)
{
	this->fwd_kin = fwd_kin;

	for(int i=0; i<NB_LEGS; i++)
	{
		feet_av_compute[i] = new RunningAverage(SIZE_FEET_FORCES_VEC, inputs->get_Fz_feet(i));
		toe_av_compute[i] = new RunningAverage(SIZE_FEET_FORCES_VEC, inputs->get_Fz_toe(i));
	}
}

/*! \brief destructor
 */
HandleInputs::~HandleInputs()
{
	for(int i=0; i<NB_LEGS; i++)
	{
		delete feet_av_compute[i];
		delete toe_av_compute[i];
	}
}

/*! \brief computations
 */
void HandleInputs::compute()
{
	feet_forces();
	toe_forces();
}

/*! \brief feet forces limited and with running average low-pass filter
 */
void HandleInputs::feet_forces()
{
	// Vertical forces [N]
	inputs->set_Fz_feet(R_ID, limit_range( fabs(inputs->get_F_Rfoot_IF(2)), -MAX_EXT_FORCE_Z_CONTROLLER, MAX_EXT_FORCE_Z_CONTROLLER) );
	inputs->set_Fz_feet(L_ID, limit_range( fabs(inputs->get_F_Lfoot_IF(2)), -MAX_EXT_FORCE_Z_CONTROLLER, MAX_EXT_FORCE_Z_CONTROLLER) );

	// mean forces
	for(int i=0; i<NB_LEGS; i++)
	{
		inputs->set_mean_Fz_feet(i, feet_av_compute[i]->update_and_get(inputs->get_Fz_feet(i)));
	}
}

/*! \brief feet forces limited and with running average low-pass filter
 */
void HandleInputs::toe_forces()
{
	// Vertical forces [N]
	inputs->set_Fz_toe(R_ID, limit_range( fabs(inputs->get_F_Rtoe_IF(2)), -MAX_EXT_FORCE_Z_CONTROLLER, MAX_EXT_FORCE_Z_CONTROLLER) );
	inputs->set_Fz_toe(L_ID, limit_range( fabs(inputs->get_F_Ltoe_IF(2)), -MAX_EXT_FORCE_Z_CONTROLLER, MAX_EXT_FORCE_Z_CONTROLLER) );

	// mean forces
	for(int i=0; i<NB_LEGS; i++)
	{
		inputs->set_mean_Fz_toe(i, toe_av_compute[i]->update_and_get(inputs->get_Fz_toe(i)));
	}
}
