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
    g_osc->opti_set_P_tau(0.04643686 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02561703 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.20878876 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.21633594);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.73031651);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 2.38454170);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.09273268 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.81148808 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 13.58527715 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.47073333 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 3.94031898);	// Prevent hyperextension
}
