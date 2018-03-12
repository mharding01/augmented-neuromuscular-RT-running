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
    g_osc->opti_set_P_tau(0.04379050 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03017403 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.21174469 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.83080591);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 8.15411572);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.37313021);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.78293854 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.64547264 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 1.79972499 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.46917734 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.86602279);	// Prevent hyperextension
}
//energy: 8.06985 [J/(m*kg)]
//speed : 1.6766 [m/s]
//period: 0.431438 [s]
//length: 0.723814 [m]
//one leg stance phase: 34.7702 [%]
//double support: 0 [%]
//tot flight: 30.4599 [%]
