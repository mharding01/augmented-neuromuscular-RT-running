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
    g_osc->opti_set_P_tau(0.04802716 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06392056 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.29646855 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 3.10671578);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.51259625);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.87598520);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.69475351 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.41966729 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 11.07766138 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.88120046 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.02044881);	// Prevent hyperextension
}
