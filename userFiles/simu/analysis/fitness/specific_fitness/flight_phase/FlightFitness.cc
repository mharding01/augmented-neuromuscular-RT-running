
#include "FlightFitness.hh"
#include "NicoCtrl.hh"
#include "Body.hh"
#include "WalkStates.hh"

#define DIST_FIT_START 3.0
#define MAX_FLIGHT_PER_CYCLE 0.1    // TODO: tune this value later?
#define FLIGHT_MARGIN 0.08

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 */
FlightFitness::FlightFitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);

	Body *lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));

	WalkStates *ws = static_cast<WalkStates*>(lower_body->get_walk_state());

	sw_st = static_cast<SwingStanceState*>(ws->get_state(SWING_STANCE_STATE));

	dt = mbs_data->dt0;
	time_flight = 0.0;
	flight_opti_t_start = 0.0;
	flight_per_cycle = 0.0;

	max_fitness = 250.0;
}

/*! \brief destructor
 */
FlightFitness::~FlightFitness()
{

}

/*! \brief compute variables at each time step
 */
void FlightFitness::compute()
{
	double t;

	if (mbs_data->q[1] > DIST_FIT_START)
	{
		t = mbs_data->tsim;

		if (!flight_opti_started)
		{
			flight_opti_started = 1;
			flight_opti_t_start = t;
		}
		else if ((t > flight_opti_t_start) && sw_st->is_flight_phase())
		{
			time_flight += dt;
			flight_per_cycle = time_flight/(t-flight_opti_t_start);
		}
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double FlightFitness::get_fitness()
{
	if (flight_per_cycle < MAX_FLIGHT_PER_CYCLE)
	{
		return compute_gaussian_fitness(MAX_FLIGHT_PER_CYCLE-flight_per_cycle, max_fitness, 450.0);
	}
	else
	{
		return max_fitness;
	}
	
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int FlightFitness::next_stage_unlocked()
{
	return 1;
}
