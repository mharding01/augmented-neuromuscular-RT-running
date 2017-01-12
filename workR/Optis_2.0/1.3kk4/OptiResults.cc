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
    g_osc->opti_set_P_tau(0.05121438 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.09357249 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.05967711 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.37102309);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.86637993);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.31541168);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.09444934 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 1.53634116 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 12.41499715 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.85431512 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 13.57963827);	// Prevent hyperextension
}
//energy: 9.87526 [J/(m*kg)]
//speed : 1.31473 [m/s]
//period: 0.481414 [s]
//length: 0.634628 [m]
//one leg stance phase: 31.1683 [%]
//double support: 0 [%]
//tot flight: 37.638 [%]

