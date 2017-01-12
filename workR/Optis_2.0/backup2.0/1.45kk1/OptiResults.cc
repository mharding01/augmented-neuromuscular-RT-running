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
    g_osc->opti_set_P_tau(0.04817830 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04335383 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.27207872 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.06231033);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.17635992);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 6.09991341);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.17628623 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 2.77692420 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 17.07895435 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.10289265 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.66257301);	// Prevent hyperextension
}
//energy: 8.57792 [J/(m*kg)]
//speed : 1.43668 [m/s]
//period: 0.461015 [s]
//length: 0.664119 [m]
//one leg stance phase: 32.5386 [%]
//double support: 0 [%]
//tot flight: 34.9326 [%]

