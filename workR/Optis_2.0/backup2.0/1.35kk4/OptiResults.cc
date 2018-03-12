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
    g_osc->opti_set_P_tau(0.04787969 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.04227109 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.03034163 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 4.16545673);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 1.74302445);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 3.11366258);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.71645637 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 3.91628008 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 18.57304327 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.96255787 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 2.22250420);	// Prevent hyperextension
}
//energy: 9.2441 [J/(m*kg)]
//speed : 1.36168 [m/s]
//period: 0.463988 [s]
//length: 0.632072 [m]
//one leg stance phase: 32.3314 [%]
//double support: 0 [%]
//tot flight: 35.332 [%]

