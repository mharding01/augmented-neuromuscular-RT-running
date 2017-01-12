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
    g_osc->opti_set_P_tau(0.04576023 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07070763 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.01786912 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.46245867);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 7.00723846);	// Higher bound
    g_osc->opti_set_k_HAMrun( 1.45324647);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.34205119 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.46559866 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 8.85463810 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.59042102 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 14.29222865);	// Prevent hyperextension
}
