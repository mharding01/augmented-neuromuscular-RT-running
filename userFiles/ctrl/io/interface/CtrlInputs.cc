
#include "CtrlInputs.hh"
#include "OptiResults.hh"
#include "OptiGeneration.hh"

/*! \brief constructor
 */
CtrlInputs::CtrlInputs(UserCtrl *user_ctrl, MotorCtrlIndex *ctrl_index, CtrlOptions *options)
{
	this->options = options;

	nb_mot = ctrl_index->get_nb_mot();

	t = 0.0;

	for(int i=0; i<nb_mot; i++)
	{
		q.push_back(0.0);
		qd.push_back(0.0);
		Qq.push_back(0.0);
		q_mot.push_back(0.0);
		qd_mot.push_back(0.0);

		q_ref.push_back(0.0);
		Qq_ref.push_back(0.0);
	}

	for(int i=0; i<2; i++)
	{
		Fz_feet[i] = 0.0;
	}

	for(int i=0; i<3; i++)
	{
		F_Rfoot[i] = 0.0;
		F_Lfoot[i] = 0.0;
		T_Rfoot[i] = 0.0;
		T_Lfoot[i] = 0.0;

		F_Rfoot_IF[i] = 0.0;
		F_Lfoot_IF[i] = 0.0;
		F_Rtoe_IF[i] = 0.0;
		F_Ltoe_IF[i] = 0.0;
		T_Rfoot_IF[i] = 0.0;
		T_Lfoot_IF[i] = 0.0;

		theta_waist[i] = 0.0;
		theta_torso[i] = 0.0;
		theta_Rfoot[i] = 0.0;
		theta_Lfoot[i] = 0.0;

		omega_waist[i] = 0.0;
		omega_torso[i] = 0.0;
		omega_Rfoot[i] = 0.0;
		omega_Lfoot[i] = 0.0;

		IMU_Angular_Rate[i] = 0.0;
		IMU_Acceleration[i] = 0.0;
	}

	for(int i=0; i<9; i++)
	{
		IMU_Orientation[i] = 0.0;
	}

	for(int i=0; i<NB_LEGS; i++)
	{
		mean_Fz_feet[i] = 0.0;
	}

	// optimization parameters
	if (options->is_opti())
	{
		opti_inputs = new OptiGeneration();
	}
	else
	{
		opti_inputs = new OptiResults();
	}

	// user control
	this->user_ctrl = user_ctrl;

	// motor indexes
	this->ctrl_index = ctrl_index;
}

/*! \brief destructor
 */
CtrlInputs::~CtrlInputs()
{
	delete opti_inputs;
}
