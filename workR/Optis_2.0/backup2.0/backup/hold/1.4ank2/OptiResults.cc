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
    g_osc->opti_set_P_tau(0.04599718 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06421533 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.16624026 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.75730980);
    g_osc->opti_set_k_HFLrun2( 7.48374118);
    g_osc->opti_set_k_HAMrun( 4.39459489);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.48264339 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 4.25779652 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.39345012 );
}
