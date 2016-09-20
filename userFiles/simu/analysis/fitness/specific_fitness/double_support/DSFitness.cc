
#include "DSFitness.hh"
#include "NicoCtrl.hh"
#include "Body.hh"
#include "WalkStates.hh"

#define DIST_DS_START 5.0

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
DSFitness::DSFitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);

	Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));

	WalkStates *ws = static_cast<WalkStates*>(lower_body->get_walk_state());

	sw_st = static_cast<SwingStanceState*>(ws->get_state(SWING_STANCE_STATE));

	dt = mbs_data->dt0;
	time_ds = 0.0;
	ds_opti_t_start = 0.0;
	ds_per_cycle = 0.0;

	max_fitness = 100.0;
}

/*! \brief destructor
 */
DSFitness::~DSFitness()
{

}

/*! \brief compute variables at each time step
 */
void DSFitness::compute()
{
	double t;

	if (mbs_data->q[1] > DIST_DS_START)
	{
		t = mbs_data->tsim;

		if (!ds_opti_started)
		{
			ds_opti_started = 1;
			ds_opti_t_start = t;
		}
		else if ((t > ds_opti_t_start) && sw_st->is_double_support())
		{
			time_ds += dt;
			ds_per_cycle = time_ds/(t-ds_opti_t_start);
		}
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double DSFitness::get_fitness()
{
	return compute_gaussian_fitness(ds_per_cycle, max_fitness, 150.0);
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int DSFitness::next_stage_unlocked()
{
	return 1;
}
