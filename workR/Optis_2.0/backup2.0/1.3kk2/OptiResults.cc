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
    g_osc->opti_set_P_tau(0.05125521 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.10945943 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.02966603 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.40963458);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.42887729);	// Higher bound
    g_osc->opti_set_k_HAMrun( 2.62212757);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.13113797 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 0.48796004 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 16.85871988 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.93225200 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.69404066);	// Prevent hyperextension
}
//energy: 9.95932 [J/(m*kg)]
//speed : 1.31824 [m/s]
//period: 0.483315 [s]
//length: 0.634769 [m]
//one leg stance phase: 31.0384 [%]
//double support: 0 [%]
//tot flight: 37.9153 [%]

