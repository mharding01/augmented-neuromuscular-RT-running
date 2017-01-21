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
    g_osc->opti_set_P_tau(0.04650797 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02579637 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.11920191 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.75638307);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.80804218);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.49073096);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.73437618 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.39346812 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 16.38232312 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.92271206 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 4.70408449);	// Prevent hyperextension
}
