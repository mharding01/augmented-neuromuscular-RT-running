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
    g_osc->opti_set_P_tau(0.04593035 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.00612768 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.07440046 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.79745696);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.49648733);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 2.41310208);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.42298671 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.78274231 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.31336694 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.49734802 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.27945264);	// Prevent hyperextension
}
