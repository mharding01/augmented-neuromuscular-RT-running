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
    g_osc->opti_set_P_tau(0.04682965 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06882829 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.24861891 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.20867566);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 6.62274116);	// Higher bound
    g_osc->opti_set_k_HAMrun( 2.94125635);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.62610391 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.81708802 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 1.30649368 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.65049051 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.47458527);	// Prevent hyperextension
}
//energy: 9.83678 [J/(m*kg)]
//speed : 1.29193 [m/s]
//period: 0.449588 [s]
//length: 0.58054 [m]
//one leg stance phase: 33.3669 [%]
//double support: 0 [%]
//tot flight: 33.2614 [%]
