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
    g_osc->opti_set_P_tau(0.04841582 ); // Taken from master commit: "All_1 results, 1459 score"
	g_osc->opti_set_P_theta_trunk( 0.00779141 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.49503703 ); // TODO: made its lower bound smaller, come back to this later
	g_osc->opti_set_P_G_SOL( 1.30647761 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 5.16471636 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 17.78797636 );

    g_osc->opti_set_k_HFLrun1( 0.85954847);
    g_osc->opti_set_k_HFLrun2( 8.88573767);
    g_osc->opti_set_k_HAMrun3( 1.55594357);
}
