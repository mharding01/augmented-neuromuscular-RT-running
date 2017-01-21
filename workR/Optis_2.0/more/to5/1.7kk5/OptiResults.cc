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
    g_osc->opti_set_P_tau(0.04696247 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07868254 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.28786553 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 7.68388853);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 9.16743073);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.09313109);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 4.06628125 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.16513754 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.13135982 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.28022968 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.52784893);	// Prevent hyperextension
}
