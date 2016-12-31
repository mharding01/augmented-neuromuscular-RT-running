
#include "WalkTimeFitness.hh"

#define WT_PERCENT_THRESH 0.75
/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
WalkTimeFitness::WalkTimeFitness(MbsData *mbs_data): FitnessStage(mbs_data)
{
	tf = mbs_data->tf;
	max_fitness = 500.0;
}

/*! \brief destructor
 */
WalkTimeFitness::~WalkTimeFitness()
{

}

/*! \brief compute variables at each time step
 */
void WalkTimeFitness::compute()
{

}

/*! \brief get fitness
 * 
 * \return fitness
 */
double WalkTimeFitness::get_fitness()
{
	return mbs_data->tsim * (max_fitness / tf);
}

/*! \brief detect if next stage is unlocked
 * 
 * \return 1 if next stage unlocked, 0 otherwise
 */
int WalkTimeFitness::next_stage_unlocked()
{
	return (mbs_data->tsim / tf) > WT_PERCENT_THRESH;
}
