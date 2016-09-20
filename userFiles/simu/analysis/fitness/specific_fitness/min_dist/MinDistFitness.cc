
#include "MinDistFitness.hh"

#define THRESHOLD_WAIST 0.45

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] sens_info info from the sensor
 */
MinDistFitness::MinDistFitness(MbsData *mbs_data, SensorsInfo *sens_info): FitnessStage(mbs_data)
{
	max_x_before_fall = 0.0;

	tf = mbs_data->tf;

	lim_x_travel = 0.25 * tf;

	max_fitness = 500.0;

	this->sens_info = sens_info;
}

/*! \brief destructor
 */
MinDistFitness::~MinDistFitness()
{

}

/*! \brief compute variables at each time step
 */
void MinDistFitness::compute()
{
	double x_before_fall;

	// threshold_waist prevents from counting positive travelled distance if the robot is falling
    if (sens_info->get_waist_to_feet() > THRESHOLD_WAIST)
	{
		// no interest if going backward
		x_before_fall = fmin(fmin(sens_info->get_S_MidWaist_P(0),sens_info->get_S_RFoots_P(0)),sens_info->get_S_LFoots_P(0));

		if (x_before_fall < 0.0)
		{
			x_before_fall = 0.0;
		}
        
		// to consider only the max positive value
        if (x_before_fall > max_x_before_fall)
		{
            max_x_before_fall = x_before_fall;
		}
    }
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double MinDistFitness::get_fitness()
{
    return (max_x_before_fall > lim_x_travel) ? max_fitness : max_x_before_fall * (max_fitness/lim_x_travel);
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int MinDistFitness::next_stage_unlocked()
{
	return 1.0;
}
