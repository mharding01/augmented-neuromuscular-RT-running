#include <stdio.h>
#include "PolyRegL1Fitness.hh"
#include "NicoCtrl.hh"
#include "Body.hh"
#include "StimWalkCtrl.hh"
#include "StimWangCtrl.hh"
#include <math.h> // fabs
/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
PolyRegL1Fitness::PolyRegL1Fitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);

	Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));

	StimWangCtrl *stim_wang_ctrl = static_cast<StimWangCtrl*>(lower_body->get_stim_ctrl());

	osc = stim_wang_ctrl->get_ghost_osc();

	max_fitness = 400.0; //TODO
    
    // Compute L1-norm on construction
    l1_norm = 0.0;
    // TODO: for now, only regularize quadratic polynomial parameters
    l1_norm += fabs(osc->get_P_tau());
    l1_norm += fabs(osc->get_p_tau());
    l1_norm += fabs(osc->get_p2_tau());

    l1_norm += fabs(osc->get_P_k_HFLrun2());
    l1_norm += fabs(osc->get_p_k_HFLrun2());
    l1_norm += fabs(osc->get_p2_k_HFLrun2());

    l1_norm += fabs(osc->get_P_k_HAMrun());
    l1_norm += fabs(osc->get_p_k_HAMrun());
    l1_norm += fabs(osc->get_p2_k_HAMrun());

    l1_norm += fabs(osc->get_P_G_SOL());
    l1_norm += fabs(osc->get_p_G_SOL());
    l1_norm += fabs(osc->get_p2_G_SOL());

    fitness = compute_gaussian_fitness(l1_norm, max_fitness, 0.0002);
}

/*! \brief destructor
 */
PolyRegL1Fitness::~PolyRegL1Fitness()
{

}

/*! \brief compute variables at each time step
 */
void PolyRegL1Fitness::compute()
{
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double PolyRegL1Fitness::get_fitness()
{
    return fitness;
}


/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int PolyRegL1Fitness::next_stage_unlocked()
{
	return 1;
}
