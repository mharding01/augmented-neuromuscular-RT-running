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
    g_osc->opti_set_P_tau(0.05124832 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.08364095 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.08556161 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.99056924);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 8.81267196);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.45791045);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.16206869 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 0.89639984 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 18.05616957 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.82516731 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 2.18096853);	// Prevent hyperextension
}
//energy: 9.94933 [J/(m*kg)]
//speed : 1.32632 [m/s]
//period: 0.484107 [s]
//length: 0.640035 [m]
//one leg stance phase: 30.9883 [%]
//double support: 0 [%]
//tot flight: 38.0266 [%]

