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
    g_osc->opti_set_P_tau(0.05321516 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07502407 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.20383926 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.28778957);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.56676314);	// Higher bound
    g_osc->opti_set_k_HAMrun( 6.53526275);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.72618194 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.93313858 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 17.35553605 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 2.10583658 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 8.91626485);	// Prevent hyperextension
}
//energy: 9.76359 [J/(m*kg)]
//speed : 1.28492 [m/s]
//period: 0.498897 [s]
//length: 0.648528 [m]
//one leg stance phase: 30.0942 [%]
//double support: 0 [%]
//tot flight: 39.8268 [%]

