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
    g_osc->opti_set_P_tau(0.04658449 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07155197 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.01026216 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.25021243);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.38056712);	// Higher bound
    g_osc->opti_set_k_HAMrun( 3.26898201);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.34770119 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.68132380 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.55336044 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.13026520 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 1.25308477);	// Prevent hyperextension
}
