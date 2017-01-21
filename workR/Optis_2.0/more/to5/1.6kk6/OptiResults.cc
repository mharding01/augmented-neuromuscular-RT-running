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
    g_osc->opti_set_P_tau(0.04642930 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.05187108 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.02710013 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.15458833);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.04379145);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.77894498);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.61990930 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.48766068 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 4.33702995 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.27193538 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 12.93124780);	// Prevent hyperextension
}
