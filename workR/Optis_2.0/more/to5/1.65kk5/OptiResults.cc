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
    g_osc->opti_set_P_tau(0.04615628 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07181839 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.16554536 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.27495734);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 6.15506725);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.31305220);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.86968445 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.55142752 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 15.34722855 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.40461886 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.03857988);	// Prevent hyperextension
}
