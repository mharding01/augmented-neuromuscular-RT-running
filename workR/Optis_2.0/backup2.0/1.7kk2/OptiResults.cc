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
    g_osc->opti_set_P_tau(0.04413630 ); // Taken from master commit: "All_1 results, 1459 score"
	// HIP CONTROL
	g_osc->opti_set_P_theta_trunk( 0.03520887 ); // TODO: trunk lean 
	g_osc->opti_set_P_theta_hip( 0.01714998 ); // TODO: made its lower bound smaller, come back to this later
    
	// CPG CONTROL
    g_osc->opti_set_k_HFLrun1( 6.33766810);	// Higher bound
    g_osc->opti_set_k_HFLrun2( 10.18961249);	// Higher bound
    g_osc->opti_set_k_HAMrun( 3.63422527);

	// ANKLE CONTROL
	g_osc->opti_set_P_G_SOL( 3.64874894 );	// Lower bounds
	g_osc->opti_set_P_G_SOL_TA( 5.20565414 );// Prevent tripping
	g_osc->opti_set_P_G_GAS( 2.88794415 );
	
	// KNEE CONTROL
	g_osc->opti_set_P_G_VAS( 1.17217439 );	// Shock absorption, v. thrust
	g_osc->opti_set_P_k_theta( 7.56080964);	// Prevent hyperextension
}
//energy: 7.99943 [J/(m*kg)]
//speed : 1.695 [m/s]
//period: 0.434943 [s]
//length: 0.739874 [m]
//one leg stance phase: 34.4876 [%]
//double support: 0 [%]
//tot flight: 31.0262 [%]

