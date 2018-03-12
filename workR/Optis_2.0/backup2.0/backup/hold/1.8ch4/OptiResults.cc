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
    g_osc->opti_set_P_tau(0.04661076 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06846144 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.28678725 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.07720621);
    g_osc->opti_set_k_HFLrun2( 8.65455082);
    g_osc->opti_set_k_HAMrun( 3.57885020);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.53607449 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 6.69468894 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 6.67300839 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 3.07398072 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 4.11776313);	// Prevent hyperextension
}
