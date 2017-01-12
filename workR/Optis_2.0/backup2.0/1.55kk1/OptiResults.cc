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
    g_osc->opti_set_P_tau(0.04732165 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07563455 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.19477769 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.11484166);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.28439520);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 4.16777861);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.21234491 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.60001985 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 4.83498869 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.84880598 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 6.33898434);	// Prevent hyperextension
}
//energy: 8.61205 [J/(m*kg)]
//speed : 1.57497 [m/s]
//period: 0.4492 [s]
//length: 0.705871 [m]
//one leg stance phase: 33.3934 [%]
//double support: 0 [%]
//tot flight: 33.2078 [%]


