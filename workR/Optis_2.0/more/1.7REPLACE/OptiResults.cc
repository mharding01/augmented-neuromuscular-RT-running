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
    g_osc->opti_set_P_tau(0.04304472 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.02970069 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.04674716 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.78216757);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 11.32556380);	// Higher bound
    g_osc->opti_set_k_HAMrun( 2.22622544);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.82440815 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 6.07124276 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 16.91076550 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 3.63832465 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 10.68150210);	// Prevent hyperextension
}
//energy: 8.4321 [J/(m*kg)]
//speed : 1.6611 [m/s]
//period: 0.434579 [s]
//length: 0.719787 [m]
//one leg stance phase: 34.5168 [%]
//double support: 0 [%]
//tot flight: 30.9634 [%]

