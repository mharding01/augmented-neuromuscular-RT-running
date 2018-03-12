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
    g_osc->opti_set_P_tau(0.05421157 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.11878751 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.02371436 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.16787034);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.68547359);	// Higher bound
    g_osc->opti_set_k_HAMrun( 1.68500992);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.20282894 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 0.68854509 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 10.63668050 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.25842908 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.96287477);	// Prevent hyperextension
}
