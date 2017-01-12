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
    g_osc->opti_set_P_tau(0.05677529 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.13879736 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.08298943 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.74493077);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 8.42085414);	// Higher bound
    g_osc->opti_set_k_HAMrun( 7.46147734);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.03704646 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 0.72163104 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 9.00720684 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.45083193 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.41420417);	// Prevent hyperextension
}
