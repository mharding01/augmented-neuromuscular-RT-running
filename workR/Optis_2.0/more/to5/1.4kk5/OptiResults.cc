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
    g_osc->opti_set_P_tau(0.04945301 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06651297 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.23933812 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.33585533);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.89473476);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 2.11694304);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.39947602 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.79439835 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 14.42226774 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.18473710 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 1.34844647);	// Prevent hyperextension
}
