
#include "OscillosFitness.hh"
#include "NicoCtrl.hh"
#include "Body.hh"
#include "StimWalkCtrl.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
OscillosFitness::OscillosFitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);

	Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));

	StimWalkCtrl *stim_walk_ctrl = static_cast<StimWalkCtrl*>(lower_body->get_stim_ctrl());

	osc = stim_walk_ctrl->get_osc();

	max_fitness = 300.0;
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
	return compute_gaussian_fitness(osc->get_t_osc_error_mean(), max_fitness, 1100.0);
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int OscillosFitness::next_stage_unlocked()
{
	return 1;
}
