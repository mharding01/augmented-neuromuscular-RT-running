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
    g_osc->opti_set_P_tau(0.04526025 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04599454 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.20743441 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.01000054);
    g_osc->opti_set_k_HFLrun2( 8.38192250);
    g_osc->opti_set_k_HAMrun( 5.28170362);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.87282157 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 3.99957415 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 11.82010153 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.24988334 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.08254304);	// Prevent hyperextension
}
