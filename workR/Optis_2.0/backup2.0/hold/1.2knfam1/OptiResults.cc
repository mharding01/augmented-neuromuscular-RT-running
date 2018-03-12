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
    g_osc->opti_set_P_tau(0.05686471 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05259481 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.17206306 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.94418543);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.12161376);	// Higher bound
    g_osc->opti_set_k_HAMrun( 4.45673683);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.39141019 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.75804707 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 13.14083543 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.59968522 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 3.65394251);	// Prevent hyperextension
}
