#include "StimQqRefCtrl.hh"

/*! \brief constructor
 */
StimQqRefCtrl::StimQqRefCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options):
	StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	flag_3D = options->is_flag_3D();
}

/*! \brief destructor
 */
StimQqRefCtrl::~StimQqRefCtrl()
{

}

/*! \brief stimulations main computation
 */
void StimQqRefCtrl::compute()
{
	// minimal stimulations
	for(int i=0; i<NB_LEGS; i++)
	{
		for(unsigned j=0; j<Stim[i].size(); j++)
		{
			Stim[i][j] = S_MIN;
		}
	}

	for (int i=0; i<NB_LEGS; i++)
	{
		parts[i]->set_Qq_ref_art(); // copy Qq_ref from impedance controller
		parts[i]->update_S_ref();   // compute stimulations to fullfill Qq_ref

		for (int j=0; j<NB_LEG_MUSCLES; j++)
		{
			Stim[i][j] = parts[i]->get_S_ref(j);
		}
	}

	// 2D walking
	if (!flag_3D)
	{
		for (int i=0; i<NB_LEGS; i++)
		{
			Stim[i][HAB_MUSCLE] = S_MIN;
			Stim[i][HAD_MUSCLE] = S_MIN;
			Stim[i][EVE_MUSCLE] = S_MIN;
			Stim[i][INV_MUSCLE] = S_MIN;
			Stim[i][HER_MUSCLE] = S_MIN;
			Stim[i][HIR_MUSCLE] = S_MIN;
		}
	}
}
