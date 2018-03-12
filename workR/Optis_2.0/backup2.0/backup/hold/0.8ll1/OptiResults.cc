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
    g_osc->opti_set_P_tau(0.05989351 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03341279 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.25993771 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.00292795);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.71829499);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.06764866);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.01644945 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.95671670 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 17.80200676 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 3.50208842 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.20116877);	// Prevent hyperextension
}
