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
    g_osc->opti_set_P_tau(0.04372884 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02045432 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.29509793 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 8.81640368);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 11.25935654);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.91064550);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.42171523 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.35222815 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.80919914 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 3.36176412 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.70899061);	// Prevent hyperextension
}
//energy: 8.33445 [J/(m*kg)]
//speed : 1.67524 [m/s]
//period: 0.440017 [s]
//length: 0.740882 [m]
//one leg stance phase: 34.0909 [%]
//double support: 0 [%]
//tot flight: 31.8277 [%]
