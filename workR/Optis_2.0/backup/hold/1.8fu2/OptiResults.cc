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
    g_osc->opti_set_P_tau(0.04640111 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.10281856 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.20104515 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.80202584);
    g_osc->opti_set_k_HFLrun2( 9.51856731);
    g_osc->opti_set_k_HAMrun( 2.50049893);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.49948093 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 6.72202724 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 13.23273518 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.63347735 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 13.64882264);	// Prevent hyperextension
}
