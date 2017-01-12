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
    g_osc->opti_set_P_tau(0.04909907 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05102411 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.13155027 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.65065297);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.79404674);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.75530094);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.76469992 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.38157816 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 8.57989751 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.89974465 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.52232878);	// Prevent hyperextension
}
//energy: 8.8429 [J/(m*kg)]
//speed : 1.37489 [m/s]
//period: 0.468433 [s]
//length: 0.643967 [m]
//one leg stance phase: 32.0222 [%]
//double support: 0 [%]
//tot flight: 35.9481 [%]
