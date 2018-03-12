#include <stdio.h>
#include "OscillosFitness.hh"
#include "NicoCtrl.hh"
#include "Body.hh"
#include "StimWalkCtrl.hh"
#include "StimWangCtrl.hh"

#define DIST_OSCILLO_START .5    // 3m distance before penalizing oscillo error 

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
OscillosFitness::OscillosFitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);

	Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));

	StimWangCtrl *stim_wang_ctrl = static_cast<StimWangCtrl*>(lower_body->get_stim_ctrl());

	osc = stim_wang_ctrl->get_ghost_osc();

	max_fitness = 500.0;

}

/*! \brief destructor
 */
OscillosFitness::~OscillosFitness()
{

}

/*! \brief compute variables at each time step
 */
void OscillosFitness::compute()
{

}

/*! \brief get fitness
 * 
 * \return fitness
 */
double OscillosFitness::get_fitness()
{
    if (mbs_data->q[1] > DIST_OSCILLO_START) 
    {
        return compute_gaussian_fitness(osc->get_t_osc_error_mean(), max_fitness, 1100.0);

    }
    else
    {
        return 0.0;
    }
}


/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int OscillosFitness::next_stage_unlocked()
{
	return 1;
}
