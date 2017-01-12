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
    g_osc->opti_set_P_tau(0.04712846 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06613668 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.20988978 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.69765726);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 12.33740766);	// Higher bound
    g_osc->opti_set_k_HAMrun( 7.53574273);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.61616309 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.88246137 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.44039349 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.85032169 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 3.03271574);	// Prevent hyperextension
}
