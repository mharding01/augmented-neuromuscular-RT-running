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
    g_osc->opti_set_P_tau(0.04880825 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02960195 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.20088177 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.64955718);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 10.39983904);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.28712135);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 0.91077071 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.95583479 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.73499443 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.02496799 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 5.71737496);	// Prevent hyperextension
}
