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
    g_osc->opti_set_P_tau(0.04767456 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06078599 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.30805790 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 2.98710728);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.36627485);	// Higher bound
    g_osc->opti_set_k_HAMrun( 2.82408940);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.76805516 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.31789281 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 1.87631094 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.03355411 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.08212871);	// Prevent hyperextension
}
//energy: 8.1949 [J/(m*kg)]
//speed : 1.50051 [m/s]
//period: 0.455077 [s]
//length: 0.681065 [m]
//one leg stance phase: 32.9623 [%]
//double support: 0 [%]
//tot flight: 34.0821 [%]

