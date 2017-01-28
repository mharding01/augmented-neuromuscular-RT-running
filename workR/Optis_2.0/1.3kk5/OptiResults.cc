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
    g_osc->opti_set_P_tau(0.04928448 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03117565 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.17046441 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.40083962);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.55209174);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.86921080);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.75870291 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 2.46424917 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 7.05279588 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.97410256 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.16975527);	// Prevent hyperextension
}
//energy: 8.84057 [J/(m*kg)]
//speed : 1.31623 [m/s]
//period: 0.472363 [s]
//length: 0.62152 [m]
//one leg stance phase: 31.7667 [%]
//double support: 0 [%]
//tot flight: 36.4797 [%]
