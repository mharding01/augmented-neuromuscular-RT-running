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
    g_osc->opti_set_P_tau(0.04743856 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.01763216 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.07938568 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 2.25615507);
    g_osc->opti_set_k_HFLrun2( 3.88478191);
    g_osc->opti_set_k_HAMrun( 3.52627327);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.19127825 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 1.98050315 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 11.34666128 );
}
