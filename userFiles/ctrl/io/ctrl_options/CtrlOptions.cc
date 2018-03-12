#include "CtrlOptions.hh"

/*! \brief controller
 */
CtrlOptions::CtrlOptions()
{
	// COMAN model
	flag_coman = COMAN_SHORT_FEET;

	// controller choice
	flag_ctrl = CTRL_REFLEX_WANG; 

	// find params to follows given torques (with SIMU_Qq_MATCH_WANG in SimuOPtions)
	flag_Qq_match_wang = 0;

	// apply Wang torque (with opti shifting and sclaling) in leg pitch articulations
	apply_Qq_wang = 0;

	// start with opti init pose
	flag_initial_pos = 1;

	// 3D walking
	flag_3D = 0;

	// first leg in swing
	r_first_swing = 1;

	// upper-body motion
	upper_motion = 1;

	// controller in two parts (first with konw results)
	ctrl_two_parts = 0;

	// limiting output torques
	torque_limit = 0;

	// extra contribution for swing knee
	extra_knee = 0;

	// CPG speed range
	cpg_range = 1;

	// ham extra reflex
	extra_ham_reflex = 0;

	// ta extra reflex
	extra_ta_reflex = 0;

	// 1 to run an optimization, 0 otherwise
	#ifdef OPTI_RUN
		opti  = 1;
	#else
		opti  = 0;
	#endif

	// print in the console
	print = opti ? 0 : 1;

	//noise on the muscle stimulations
	muscle_noise = opti ? 1 : 0;
	if (flag_Qq_match_wang)
	{
		muscle_noise = 0;
	}
}

/*! \brief destructor
 */
CtrlOptions::~CtrlOptions()
{

}
