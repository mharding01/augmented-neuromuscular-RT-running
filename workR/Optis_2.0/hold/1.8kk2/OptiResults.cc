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
    g_osc->opti_set_P_tau(0.04488424 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03846552 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.15812166 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.22324197);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 8.39460663);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.41451669);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.92756453 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 4.17996102 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.99962949 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.07047256 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 11.07187106);	// Prevent hyperextension
}
