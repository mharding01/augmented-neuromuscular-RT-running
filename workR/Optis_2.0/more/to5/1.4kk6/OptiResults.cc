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
    g_osc->opti_set_P_tau(0.04925337 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05317582 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.26766577 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 8.07706306);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.64812637);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 5.76141003);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.18579107 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.12717827 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.31190960 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.57170539 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.81069843);	// Prevent hyperextension
}
