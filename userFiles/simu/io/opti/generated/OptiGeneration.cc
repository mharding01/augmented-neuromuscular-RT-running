#include "OptiGeneration.hh"
#include "StimWangCtrl.hh"
#include "JointsInit.hh"
#include "MatsuokaSixN.hh"

/*! \brief constructor
 */
OptiGeneration::OptiGeneration()
{

}

/*! \brief destructor
 */
OptiGeneration::~OptiGeneration()
{

}

/*! \brief set optimization parameters
 */
void OptiGeneration::set_opti()
{
    StimWangCtrl *stim_wang = static_cast<StimWangCtrl*>(stim_ctrl);
    MatsuokaSixN *g_osc = static_cast<MatsuokaSixN*>(stim_wang->get_ghost_osc());

    // Uses "opti_set" function family in order to delay setting these params
    // until after Ctrl construction
    g_osc->opti_set_P_tau(optiParams[0] ); // Taken from master commit: "All_1 results, 1459 score"
	g_osc->opti_set_P_theta_trunk( optiParams[1] ); // TODO: trunk lean, realistic max i pi/15=18deg.
	g_osc->opti_set_P_theta_hip( optiParams[2] ); // TODO: made its lower bound smaller, come back to this later

    g_osc->opti_set_k_HFLrun1( optiParams[3]);
    g_osc->opti_set_k_HFLrun2( optiParams[4]);
    g_osc->opti_set_k_HAMrun3( optiParams[5]);
}
