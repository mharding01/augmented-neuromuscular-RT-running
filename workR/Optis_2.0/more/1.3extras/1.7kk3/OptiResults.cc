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
    g_osc->opti_set_P_tau(0.05489336 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.13056335 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.10827089 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.00406452);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.64298060);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.59263433);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.29315588 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.47401991 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 10.09806935 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.52799456 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 5.79573134);	// Prevent hyperextension
}
