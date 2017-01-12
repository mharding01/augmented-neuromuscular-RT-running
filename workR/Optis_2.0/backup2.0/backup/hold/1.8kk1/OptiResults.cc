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
    g_osc->opti_set_P_tau(0.04453488 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04406716 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.22768451 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.53917502);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 11.17860867);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.74792209);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.79440883 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.62540746 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 5.62156875 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.60842766 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 9.46155048);	// Prevent hyperextension
}
