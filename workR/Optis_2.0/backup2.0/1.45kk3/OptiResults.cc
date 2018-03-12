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
    g_osc->opti_set_P_tau(0.04931185 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.09424237 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.19646395 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.18535698);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 5.13842660);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.26759075);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.65354174 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 1.23316421 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 4.16759273 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.16705638 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 14.61004689);	// Prevent hyperextension
}
//energy: 9.18327 [J/(m*kg)]
//speed : 1.47141 [m/s]
//period: 0.465474 [s]
//length: 0.686419 [m]
//one leg stance phase: 32.2278 [%]
//double support: 0 [%]
//tot flight: 35.5428 [%]

