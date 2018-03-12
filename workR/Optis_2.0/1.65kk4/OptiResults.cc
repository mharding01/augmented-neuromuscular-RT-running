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
    g_osc->opti_set_P_tau(0.04643808 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07040417 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.16417536 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.11375270);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 11.24216238);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.83255895);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.94073409 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.38001136 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 9.91728082 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 3.25041618 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.85835690);	// Prevent hyperextension
}
//energy: 8.82498 [J/(m*kg)]
//speed : 1.64213 [m/s]
//period: 0.444989 [s]
//length: 0.731041 [m]
//one leg stance phase: 33.7095 [%]
//double support: 0 [%]
//tot flight: 32.5731 [%]
