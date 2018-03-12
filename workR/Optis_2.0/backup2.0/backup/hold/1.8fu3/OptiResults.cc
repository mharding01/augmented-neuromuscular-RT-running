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
    g_osc->opti_set_P_tau(0.04617470 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07064170 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.32810061 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.83762955);
    g_osc->opti_set_k_HFLrun2( 7.12886957);
    g_osc->opti_set_k_HAMrun( 5.65582286);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.47372413 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 5.70508714 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 5.79540200 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.16498712 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.30961562);	// Prevent hyperextension
}
