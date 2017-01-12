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
    g_osc->opti_set_P_tau(0.04898254 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07224044 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.13937885 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.78595332);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 9.14549129);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.80457241);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.34019326 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.36294907 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 4.31722341 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.86462913 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.86894167);	// Prevent hyperextension
}
//energy: 8.93701 [J/(m*kg)]
//speed : 1.47348 [m/s]
//period: 0.466215 [s]
//length: 0.685585 [m]
//one leg stance phase: 32.1768 [%]
//double support: 0 [%]
//tot flight: 35.6458 [%]

