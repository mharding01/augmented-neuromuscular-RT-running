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
    g_osc->opti_set_P_tau(0.04723444 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.08368942 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.38171327 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.98144702);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 11.97459505);	// Higher bound
    g_osc->opti_set_k_HAMrun( 0.41546307);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.26441107 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.11420404 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 8.84421045 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.86596762 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.70199326);	// Prevent hyperextension
}
//energy: 8.78529 [J/(m*kg)]
//speed : 1.23848 [m/s]
//period: 0.50095 [s]
//length: 0.619895 [m]
//one leg stance phase: 29.9435 [%]
//double support: 0 [%]
//tot flight: 40.1199 [%]

