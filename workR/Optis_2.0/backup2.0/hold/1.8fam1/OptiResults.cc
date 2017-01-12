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
    g_osc->opti_set_P_tau(0.04108171 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02537154 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.18461737 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.59799783);
    g_osc->opti_set_k_HFLrun2( 7.09755982);
    g_osc->opti_set_k_HAMrun( 5.19556559);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.74440647 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 5.28113905 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 12.83642525 );
}
