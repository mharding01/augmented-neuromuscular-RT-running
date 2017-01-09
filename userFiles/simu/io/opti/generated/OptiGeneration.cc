#include "OptiGeneration.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OptiGeneration::OptiGeneration()
{

}

/*! \brief destructor
 */
OptiGeneration::~OptiGeneration()
{

}

/*! \brief set optimization parameters
 */
void OptiGeneration::set_opti()
{
    StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Uses "opti_set" function family in order to delay setting these params
    // until after Ctrl construction
    g_osc->opti_set_P_tau(optiParams[0] ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( optiParams[1] ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( optiParams[2] ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( optiParams[3]);
    g_osc->opti_set_k_HFLrun2( optiParams[4]);
    g_osc->opti_set_k_HAMrun3( optiParams[5]);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( optiParams[6] );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( optiParams[7] );// Prevent tripping
	g_osc->opti_set_P_G_GAS( optiParams[8] );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( optiParams[9] );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( optiParams[10]);	// Prevent hyperextension
}
