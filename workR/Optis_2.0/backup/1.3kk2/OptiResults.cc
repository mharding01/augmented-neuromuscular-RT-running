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
    g_osc->opti_set_P_tau(0.04603408 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06519859 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.09122830 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 2.99230415);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 6.49894302);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.16770959);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.41455515 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 2.56126023 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 8.43028784 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.64130876 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.11670821);	// Prevent hyperextension
}
//energy: 10.2181 [J/(m*kg)]
//speed : 1.30048 [m/s]
//period: 0.437076 [s]
//length: 0.583608 [m]
//one leg stance phase: 34.3294 [%]
//double support: 0 [%]
//tot flight: 31.3435 [%]
