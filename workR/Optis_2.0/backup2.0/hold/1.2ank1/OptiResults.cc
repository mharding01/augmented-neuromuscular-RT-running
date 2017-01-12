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
    g_osc->opti_set_P_tau(0.04588777 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05045859 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.29372607 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.70174239);
    g_osc->opti_set_k_HFLrun2( 9.34546183);
    g_osc->opti_set_k_HAMrun( 2.11836038);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.87424798 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 3.00590587 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 16.99050616 );
}
