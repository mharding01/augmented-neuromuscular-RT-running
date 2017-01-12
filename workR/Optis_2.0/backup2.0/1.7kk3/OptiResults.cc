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
    g_osc->opti_set_P_tau(0.04494773 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03601503 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.19827299 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 3.30549934);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 8.65432868);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.12264336);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.32242539 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.91083437 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.72188661 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.26380757 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.05165150);	// Prevent hyperextension
}
//energy: 8.04538 [J/(m*kg)]
//speed : 1.71022 [m/s]
//period: 0.429306 [s]
//length: 0.737815 [m]
//one leg stance phase: 34.941 [%]
//double support: 0 [%]
//tot flight: 30.1198 [%]


