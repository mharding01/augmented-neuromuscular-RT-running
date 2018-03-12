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
    g_osc->opti_set_P_tau(0.04664219 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06399640 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.26601044 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.06745049);
    g_osc->opti_set_k_HFLrun2( 9.66631219);
    g_osc->opti_set_k_HAMrun( 5.22220519);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.86030147 );	// TODO: Original bounds
	g_osc->opti_set_P_G_SOL_TA( 4.39815028 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.51507900 );

	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.66213313 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 5.09967986);	// Prevent hyperextension
}
