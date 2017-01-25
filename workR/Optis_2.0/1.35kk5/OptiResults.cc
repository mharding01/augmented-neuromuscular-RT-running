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
    g_osc->opti_set_P_tau(0.04665671 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.00823889 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.19315120 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.59758101);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.71684506);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.91714417);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.94981053 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 2.38489814 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 9.08324878 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.87610882 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 5.47439002);	// Prevent hyperextension
}
//energy: 8.27996 [J/(m*kg)]
//speed : 1.37323 [m/s]
//period: 0.457722 [s]
//length: 0.627689 [m]
//one leg stance phase: 32.7716 [%]
//double support: 0 [%]
//tot flight: 34.4556 [%]


