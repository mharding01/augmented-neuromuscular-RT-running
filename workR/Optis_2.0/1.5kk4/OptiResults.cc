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
    g_osc->opti_set_P_tau(0.04681520 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04963687 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.13324077 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 2.45195123);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.71442278);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.54775961);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.33568741 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.32197347 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 16.30584974 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.93955155 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.83877931);	// Prevent hyperextension
}
//energy: 8.65159 [J/(m*kg)]
//speed : 1.49113 [m/s]
//period: 0.447844 [s]
//length: 0.668313 [m]
//one leg stance phase: 33.4941 [%]
//double support: 0 [%]
//tot flight: 33.025 [%]

