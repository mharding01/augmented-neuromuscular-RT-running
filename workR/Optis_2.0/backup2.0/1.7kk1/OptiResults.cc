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
    g_osc->opti_set_P_tau(0.04482764 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.06007507 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.07158815 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.42179916);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 7.90705608);	// Higher bound
    g_osc->opti_set_k_HAMrun( 4.42411089);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.48993125 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.91111526 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 1.90910212 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.02225440 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 14.61933870);	// Prevent hyperextension
}
//energy: 8.41095 [J/(m*kg)]
//speed : 1.68574 [m/s]
//period: 0.433109 [s]
//length: 0.728979 [m]
//one leg stance phase: 34.635 [%]
//double support: 0 [%]
//tot flight: 30.7341 [%]

