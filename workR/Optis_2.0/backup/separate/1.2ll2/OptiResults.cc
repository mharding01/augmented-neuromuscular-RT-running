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
    g_osc->opti_set_P_tau(0.05649016 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.13052351 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.06498628 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.03080047);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.95995984);	// Higher bound
    g_osc->opti_set_k_HAMrun( 7.87899786);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.54499482 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.14737505 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 9.47188919 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.01542703 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.52581493);	// Prevent hyperextension
}
//energy: 10.5964 [J/(m*kg)]
//speed : 1.23106 [m/s]
//period: 0.520168 [s]
//length: 0.642069 [m]
//one leg stance phase: 28.8417 [%]
//double support: 0 [%]
//tot flight: 42.3194 [%]
