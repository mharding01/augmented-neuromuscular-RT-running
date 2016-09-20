
#include "FootFitness.hh"
#include "NicoCtrl.hh"
#include "Body.hh"
#include "WalkStates.hh"

#define DIST_FOOT_START 2.0
#define STANCE_MIN 0.6

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensor
 */
FootFitness::FootFitness(MbsData *mbs_data, SensorsInfo *sens_info, Ctrl *ctrl): FitnessStage(mbs_data)
{
	this->sens_info = sens_info;

	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);
	Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));
	WalkStates *ws = static_cast<WalkStates*>(lower_body->get_walk_state());
	sw_st = static_cast<SwingStanceState*>(ws->get_state(SWING_STANCE_STATE));

	stance_r = 0.0;
	stance_l = 0.0;
	stance_mean = 0.0;

	dt = mbs_data->dt0;

	foot_opti_started = 0;
	foot_opti_t_start = 0.0;

	max_fitness = 25.0;
}

/*! \brief destructor
 */
FootFitness::~FootFitness()
{

}

/*! \brief compute variables at each time step
 */
void FootFitness::compute()
{
	double t;

	if (mbs_data->q[1] > DIST_FOOT_START)
	{
		t = mbs_data->tsim;

		if (!foot_opti_started)
		{
			foot_opti_started = 1;
			foot_opti_t_start = t;
		}

		if (!sw_st->is_swing_leg(R_ID))
		{
			stance_r += dt;
		}
		else if (!sw_st->is_swing_leg(L_ID))
		{
			stance_l += dt;
		}

		stance_mean = (stance_r + stance_l) / (t-foot_opti_t_start);
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double FootFitness::get_fitness()
{
	if (foot_opti_started)
	{
		return compute_gaussian_fitness(stance_mean-0.6, max_fitness, 7);
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
int FootFitness::next_stage_unlocked()
{
	return 1;
}
