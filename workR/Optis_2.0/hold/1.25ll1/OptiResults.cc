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
    g_osc->opti_set_P_tau(0.05656117 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.13339033 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.03745467 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.89155746);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.96561856);	// Higher bound
    g_osc->opti_set_k_HAMrun( 4.40219576);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.29972856 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 2.80425557 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 18.90947198 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.36587146 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.07331517);	// Prevent hyperextension
}
