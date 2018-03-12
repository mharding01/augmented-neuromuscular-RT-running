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
    g_osc->opti_set_P_tau(0.04798094 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02152718 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.24913054 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.62795444);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.42895896);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 2.10968200);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.12131221 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.76331622 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 11.57630054 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.68809288 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 13.62186708);	// Prevent hyperextension
}
