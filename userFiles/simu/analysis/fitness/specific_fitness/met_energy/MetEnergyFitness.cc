#include "MetEnergyFitness.hh"
#include "NicoCtrl.hh"
#include "user_IO.h"
#include "coman_properties.hh"

#define DIST_ENERGY_START 3.0

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] min_dist_fit minimal distance fitness stage
 * \param[in] ctrl controller
 */
MetEnergyFitness::MetEnergyFitness(MbsData *mbs_data, Ctrl *ctrl): FitnessStage(mbs_data)
{
	NicoCtrl *nico_ctrl = static_cast<NicoCtrl*>(ctrl);

	lower_body = static_cast<Body*>(nico_ctrl->get_manager()->get_computation(LOWER_BODY));

	met_energy_init_flag = 0;

	met_energy_init = 0.0;
	met_energy_end  = 0.0;

	met_energy_dist = 0.0;

	dt = mbs_data->dt0;

	max_fitness = 200.0;

	options = mbs_data->user_IO->options;
	printed = 0;
}

/*! \brief destructor
 */
MetEnergyFitness::~MetEnergyFitness()
{

}

/*! \brief compute variables at each time step
 */
void MetEnergyFitness::compute()
{
	if ( (!met_energy_init_flag) && (mbs_data->q[1] > DIST_ENERGY_START))
	{
		met_energy_init_flag = 1;
		met_energy_init = lower_body->get_met_energy_total();
	}
	
	if (met_energy_init_flag)
	{
		met_energy_end = lower_body->get_met_energy_total();

		met_energy_dist = (met_energy_end - met_energy_init) / (mbs_data->q[1]- DIST_ENERGY_START);

		if ((options->print) && (mbs_data->tsim > 9.0) && (!printed))
		{
			printed = 1;
			std::cout << "energy: " << met_energy_dist/MASS_COMAN << " [J/(m*kg)]" << std::endl;
		}
	}
}

/*! \brief get fitness
 * 
 * \return fitness
 */
double MetEnergyFitness::get_fitness()
{
	if (met_energy_init_flag)
	{
		return compute_gaussian_fitness(met_energy_dist/MASS_COMAN, max_fitness, 0.01);
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
int MetEnergyFitness::next_stage_unlocked()
{
	return 1;
}
