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
    g_osc->opti_set_P_tau(0.04483003 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05300398 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.11492435 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.67483040);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 13.77699696);	// Higher bound
    g_osc->opti_set_k_HAMrun( 4.16912658);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.71037328 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.80576465 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 6.62637472 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 3.42004402 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.53999975);	// Prevent hyperextension
}
