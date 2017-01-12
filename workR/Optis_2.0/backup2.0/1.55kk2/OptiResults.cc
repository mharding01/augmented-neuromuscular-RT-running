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
    g_osc->opti_set_P_tau(0.04822707 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.09314111 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.10138530 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.26340009);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 9.97262958);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.97205806);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.71658649 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.58529739 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 17.33019831 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.35252904 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 4.39998654);	// Prevent hyperextension
}
//energy: 8.93159 [J/(m*kg)]
//speed : 1.56674 [m/s]
//period: 0.457584 [s]
//length: 0.720566 [m]
//one leg stance phase: 32.782 [%]
//double support: 0 [%]
//tot flight: 34.421 [%]

