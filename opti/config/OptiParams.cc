#include "OPTI_NAME.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OPTI_NAME::OPTI_NAME()
{

}

/*! \brief destructor
 */
OPTI_NAME::~OPTI_NAME()
{

}

/*! \brief set optimization parameters
 */
void OPTI_NAME::set_opti()
{
    StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Uses "opti_set" function family in order to delay setting these params
    // until after Ctrl construction
    g_osc->opti_set_P_tau({0.010 ; 0.15} ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( { 0.00 ; 0.25 } ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( { 0.05 ; 0.9 } ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( {0.3 ; 6.1});
    g_osc->opti_set_k_HFLrun2( {1.0 ; 10.0});
    g_osc->opti_set_k_HAMrun3( {0.3 ; 7.0});

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( { 0.97 ; 5.0 } );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( { 0.4 ; 7.0 } );// Prevent tripping
	g_osc->opti_set_P_G_GAS( { 0.0 ; 20.0 } );
}
