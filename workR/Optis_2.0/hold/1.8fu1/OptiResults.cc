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
    g_osc->opti_set_P_tau(0.04482491 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04619921 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.34744131 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 3.99063691);
    g_osc->opti_set_k_HFLrun2( 8.27879068);
    g_osc->opti_set_k_HAMrun( 3.64572575);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.57743069 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 5.00107454 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.85605849 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.13762314 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.53233975);	// Prevent hyperextension
}
