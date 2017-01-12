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
    g_osc->opti_set_P_tau(0.04757838 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.08303866 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.23150917 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.10398513);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 6.30005898);	// Higher bound
    g_osc->opti_set_k_HAMrun( 5.52816526);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.56178632 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.38289740 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 10.32014626 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.36824612 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 5.42587220);	// Prevent hyperextension
}
//energy: 8.85115 [J/(m*kg)]
//speed : 1.6212 [m/s]
//period: 0.450919 [s]
//length: 0.730231 [m]
//one leg stance phase: 33.2661 [%]
//double support: 0 [%]
//tot flight: 33.4601 [%]

