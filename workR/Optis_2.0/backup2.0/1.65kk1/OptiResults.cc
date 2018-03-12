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
    g_osc->opti_set_P_tau(0.04345471 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02884159 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.10800888 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.14980002);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 7.92942565);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.04929927);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.11240575 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.98080039 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 1.92024917 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.73831631 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 6.12731603);	// Prevent hyperextension
}
//energy: 8.44662 [J/(m*kg)]
//speed : 1.64795 [m/s]
//period: 0.432274 [s]
//length: 0.71333 [m]
//one leg stance phase: 34.7011 [%]
//double support: 0 [%]
//tot flight: 30.6076 [%]

