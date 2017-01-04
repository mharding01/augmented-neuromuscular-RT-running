#include "SpeedFitness.hh"
#include "cmake_config.h"
#include "user_IO.h"

#define DIST_VELOCITY_START 2.0
#define VELOCITY_MARGIN 0.05

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensors
 */
SpeedFitness::SpeedFitness(MbsData *mbs_data, SensorsInfo *sens_info): FitnessStage(mbs_data)
{
	this->sens_info = sens_info;

	speed_opti_started = 0;
	
	speed_opti_x_start = 0.0;
	speed_opti_t_start = 0.0;

	speed_opti = 0.0;

	max_fitness = 500.0;

	speed_ref = mbs_data->user_IO->options->opti_speed_ref;
}

/*! \brief destructor
 */
SpeedFitness::~SpeedFitness()
{

}

/*! \brief compute variables at each time step
 */
void SpeedFitness::compute()
{
	double t;

	if (mbs_data->q[1] > DIST_VELOCITY_START)
	{
		t = mbs_data->tsim;

		if (!speed_opti_started)
		{
			speed_opti_started = 1;
			speed_opti_x_start = sens_info->get_S_MidWaist_P(0);
			speed_opti_t_start = t;
			speed_opti = 0.0;
		}
		else if (t > speed_opti_t_start)
		{
			speed_opti = (sens_info->get_S_MidWaist_P(0) - speed_opti_x_start) / (t - speed_opti_t_start); // mean velocity
		}
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double SpeedFitness::get_fitness()
{
	if (speed_opti_started)
	{
		if (fabs(speed_opti - speed_ref) > VELOCITY_MARGIN)
		{
			return compute_gaussian_fitness(speed_ref - speed_opti, max_fitness, 10);
		}
		else
		{
			return max_fitness;
		}
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
int SpeedFitness::next_stage_unlocked()
{
	return 1;
}
