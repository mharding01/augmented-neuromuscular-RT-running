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
    g_osc->opti_set_P_tau(0.04778932 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06186027 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.29453457 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 3.37379374);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.18061043);	// Higher bound
    g_osc->opti_set_k_HAMrun( 1.97725067);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.95971834 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.05164843 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 4.16982243 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.35877562 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 14.88240132);	// Prevent hyperextension
}
//energy: 8.10683 [J/(m*kg)]
//speed : 1.49739 [m/s]
//period: 0.455753 [s]
//length: 0.680773 [m]
//one leg stance phase: 32.9132 [%]
//double support: 0 [%]
//tot flight: 34.1666 [%]

