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
    g_osc->opti_set_P_tau(0.04690666 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07306013 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.24879764 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.54460245);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 7.53470782);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 6.51389981);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.54035634 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.97637112 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 18.83336828 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.44975633 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.95553695);	// Prevent hyperextension
}
//energy: 8.78572 [J/(m*kg)]
//speed : 1.64418 [m/s]
//period: 0.450097 [s]
//length: 0.736541 [m]
//one leg stance phase: 33.3271 [%]
//double support: 0 [%]
//tot flight: 33.3474 [%]

