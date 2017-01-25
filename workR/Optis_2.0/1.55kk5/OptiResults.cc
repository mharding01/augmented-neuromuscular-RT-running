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
    g_osc->opti_set_P_tau(0.04701063 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03926952 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.33599722 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.59448233);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 2.01028392);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 2.65540340);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.47228571 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.28153272 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.32131068 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.40407189 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 5.23071198);	// Prevent hyperextension
}
//energy: 7.90682 [J/(m*kg)]
//speed : 1.54557 [m/s]
//period: 0.450415 [s]
//length: 0.69423 [m]
//one leg stance phase: 33.3032 [%]
//double support: 0 [%]
//tot flight: 33.3976 [%]

