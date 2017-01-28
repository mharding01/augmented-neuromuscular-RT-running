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
    g_osc->opti_set_P_tau(0.04789993 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.09117401 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.02583599 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.28600550);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 3.49959699);	// Higher bound
    g_osc->opti_set_k_HAMrun3( 1.49967874);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 2.59095565 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.83919788 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 9.36674043 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 0.85040838 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 9.82972508);	// Prevent hyperextension
}
//energy: 8.21331 [J/(m*kg)]
//speed : 1.5013 [m/s]
//period: 0.456886 [s]
//length: 0.684889 [m]
//one leg stance phase: 32.8313 [%]
//double support: 0 [%]
//tot flight: 34.3424 [%]
