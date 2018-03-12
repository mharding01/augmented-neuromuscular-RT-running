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
    g_osc->opti_set_P_tau(0.05460309 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.13112845 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.10015994 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.91208141);
    g_osc->opti_set_k_HFLrun2( 2.89076185);
    g_osc->opti_set_k_HAMrun( 3.35452555);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.67042754 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 1.60777330 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 17.60137185 );
}
