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
    g_osc->opti_set_P_tau(0.05728729 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.14007945 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.12916010 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.37400578);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 4.83902500);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.12996331);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.23621831 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 1.56269587 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 8.57911096 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.44676590 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.35307351);	// Prevent hyperextension
}
//energy: 10.8821 [J/(m*kg)]
//speed : 1.24173 [m/s]
//period: 0.52422 [s]
//length: 0.649887 [m]
//one leg stance phase: 28.6258 [%]
//double support: 0 [%]
//tot flight: 42.7151 [%]

