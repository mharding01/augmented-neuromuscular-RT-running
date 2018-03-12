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
    g_osc->opti_set_P_tau(0.04618178 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04548981 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.36141729 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.09959947);
    g_osc->opti_set_k_HFLrun2( 8.33030295);
    g_osc->opti_set_k_HAMrun( 5.78091030);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.66649708 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 5.69277502 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.53139472 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.81234514 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 4.97921321);	// Prevent hyperextension
}
