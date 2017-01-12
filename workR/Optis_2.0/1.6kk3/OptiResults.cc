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
    g_osc->opti_set_P_tau(0.04726510 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.08367378 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.16188046 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.06996487);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 5.38678565);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.51751321);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.88119218 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.51162005 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 9.83845810 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.59020686 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.71517689);	// Prevent hyperextension
}
//energy: 8.91709 [J/(m*kg)]
//speed : 1.59737 [m/s]
//period: 0.449636 [s]
//length: 0.721364 [m]
//one leg stance phase: 33.3611 [%]
//double support: 0 [%]
//tot flight: 33.2727 [%]

