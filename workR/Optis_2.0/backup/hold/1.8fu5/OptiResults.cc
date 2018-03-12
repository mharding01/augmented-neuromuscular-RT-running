#include "OptiResults.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OptiResults::OptiResults()
{

}

/*! \brief destructor
 */
OptiResults::~OptiResults()
{

}

/*! \brief set optimization parameters
 */
void OptiResults::set_opti()
{
    StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Uses "opti_set" function family in order to delay setting these params
    // until after Ctrl construction
    g_osc->opti_set_P_tau(0.04658176 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07700005 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.18642426 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.04718526);
    g_osc->opti_set_k_HFLrun2( 9.83988472);
    g_osc->opti_set_k_HAMrun( 2.03696615);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.11915914 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 4.99271170 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.98881181 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.34719114 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.28973853);	// Prevent hyperextension
}
