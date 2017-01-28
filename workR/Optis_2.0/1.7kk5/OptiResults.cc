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
    g_osc->opti_set_P_tau(0.04397186 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02880139 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.13039706 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.51322406);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 9.92821120);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.08598408);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.93023827 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.66323080 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 15.33333578 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.38283515 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.58997465);	// Prevent hyperextension
}
//energy: 8.24805 [J/(m*kg)]
//speed : 1.69125 [m/s]
//period: 0.434799 [s]
//length: 0.737614 [m]
//one leg stance phase: 34.4992 [%]
//double support: 0 [%]
//tot flight: 30.994 [%]

