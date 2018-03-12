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
    g_osc->opti_set_P_tau(0.04823693 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.08460136 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.21800487 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.03159334);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.25331376);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 6.82064761);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.93793026 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.78834205 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 14.71957706 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.70032514 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 9.42675734);	// Prevent hyperextension
}
//energy: 8.85514 [J/(m*kg)]
//speed : 1.55868 [m/s]
//period: 0.456364 [s]
//length: 0.710903 [m]
//one leg stance phase: 32.8695 [%]
//double support: 0 [%]
//tot flight: 34.2606 [%]

