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
    g_osc->opti_set_P_tau(0.04833368 ); // Taken from master commit: "All_1 results, 1459 score"
	g_osc->opti_set_P_theta_trunk( 0.08847387 ); // TODO: trunk lean, realistic max i pi/15=18deg.
	g_osc->opti_set_P_theta_hip( 0.59384559 ); // TODO: made its lower bound smaller, come back to this later

    g_osc->opti_set_k_HFLrun1( 1.00392891);
    g_osc->opti_set_k_HFLrun2( 4.85138550);
    g_osc->opti_set_k_HAMrun3( 4.68310321);
}
