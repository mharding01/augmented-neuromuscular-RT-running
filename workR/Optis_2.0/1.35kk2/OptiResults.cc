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
    g_osc->opti_set_P_tau(0.04651104 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.07579016 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.18802704 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 5.26888023);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 8.84736136);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.20126946);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 1.50527599 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 1.56662985 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 3.81221533 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.30613072 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 4.35867103);	// Prevent hyperextension
}
//energy: 9.67519 [J/(m*kg)]
//speed : 1.3706 [m/s]
//period: 0.444494 [s]
//length: 0.606208 [m]
//one leg stance phase: 33.7466 [%]
//double support: 0 [%]
//tot flight: 32.506 [%]

