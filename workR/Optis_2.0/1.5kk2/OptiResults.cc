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
    g_osc->opti_set_P_tau(0.04751023 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07843753 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.09388449 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.26168385);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 7.68095120);	// Higher bound
    g_osc->opti_set_k_HAMrun( 1.98304097);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.23258029 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.05014648 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 4.19886126 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.82307686 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 13.05833834);	// Prevent hyperextension
}
//energy: 8.4237 [J/(m*kg)]
//speed : 1.52626 [m/s]
//period: 0.454915 [s]
//length: 0.693592 [m]
//one leg stance phase: 32.9744 [%]
//double support: 0 [%]
//tot flight: 34.055 [%]


