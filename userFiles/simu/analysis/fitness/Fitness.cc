
#include "Fitness.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] ctrl controller
 * \param[in] sens_info info from the sensor
 */
Fitness::Fitness(MbsData *mbs_data, ModelSimuIndex *simu_index, Ctrl *ctrl, SensorsInfo *sens_info): ComputationSimu(mbs_data, simu_index)
{
	this->ctrl = ctrl;
	this->sens_info = sens_info;

	total_fitness = 0.0;
}

/*! \brief destructor
 */
Fitness::~Fitness()
{
	for (unsigned int i=0; i<fitness_tab.size(); i++)
	{
		delete fitness_tab[i];	
	}
}

/*! \brief compute all the fitness stages
 */
void Fitness::compute()
{
	fitness_details.clear();
	total_fitness = 0.0;

	// computation needed at each simulation time step
	for(unsigned int i=0; i<fitness_tab.size(); i++)
	{
		fitness_tab[i]->compute();
	}

	// computation of the fitness itself
	for(unsigned int i=0; i<fitness_tab.size(); i++)
	{
		fitness_details.push_back(fitness_tab[i]->get_fitness());
		total_fitness += fitness_tab[i]->get_fitness();
		// TODO
		/*if (!fitness_tab[i]->next_stage_unlocked())
		{
			break;
		}*/
	}
}

/*! \brief set speed reference
 * 
 * \param[in] value speed reference [m/s]
 */
void Fitness::set_speed_ref(double value)
{
	speed_fitness->set_speed_ref(value);
}
