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
    g_osc->opti_set_P_tau(0.05021126 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.10069678 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.00858043 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.12965248);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 5.47475019);	// Higher bound
    g_osc->opti_set_k_HAMrun( 1.49086295);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.36387240 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.68293685 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 19.06004860 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.82755868 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.16794773);	// Prevent hyperextension
}
//energy: 8.7329 [J/(m*kg)]
//speed : 1.42804 [m/s]
//period: 0.475878 [s]
//length: 0.677374 [m]
//one leg stance phase: 31.5234 [%]
//double support: 0 [%]
//tot flight: 36.953 [%]


