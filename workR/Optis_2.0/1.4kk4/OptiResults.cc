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
    g_osc->opti_set_P_tau(0.04912441 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05436483 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.19499727 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.76335780);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.72476404);	// Higher bound
    g_osc->opti_set_k_HAMrun( 2.47472245);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.82182164 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.65030000 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 10.17038497 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.02810699 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 1.92512670);	// Prevent hyperextension
}
//energy: 8.55023 [J/(m*kg)]
//speed : 1.40221 [m/s]
//period: 0.468613 [s]
//length: 0.655757 [m]
//one leg stance phase: 32.0107 [%]
//double support: 0 [%]
//tot flight: 35.9527 [%]

