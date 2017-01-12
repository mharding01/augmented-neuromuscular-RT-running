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
    g_osc->opti_set_P_tau(0.04932945 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.08843523 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.09295294 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.10871130);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 4.86677959);	// Higher bound
    g_osc->opti_set_k_HAMrun( 2.69182442);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.63546805 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 1.28842555 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 6.93858723 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.87406490 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.26278981);	// Prevent hyperextension
}
//energy: 9.29217 [J/(m*kg)]
//speed : 1.42664 [m/s]
//period: 0.467462 [s]
//length: 0.666522 [m]
//one leg stance phase: 32.0897 [%]
//double support: 0 [%]
//tot flight: 35.8266 [%]

