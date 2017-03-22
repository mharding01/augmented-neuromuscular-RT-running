/*! 
 * \author Nicolas Van der Noot
 * \file Fitness.hh
 * \brief Fitness class
 */

#ifndef _FITNESS_HH_
#define _FITNESS_HH_

#include "ComputationSimu.hh"
#include "Ctrl.hh"
#include "FitnessStage.hh"
#include "SensorsInfo.hh"
#include "SpeedFitness.hh"

#include <vector>

/*! \brief Computation of the fitness function for optimization
 */
class Fitness: public ComputationSimu
{
	public:
		Fitness(MbsData *mbs_data, ModelSimuIndex *simu_index, Ctrl *ctrl, SensorsInfo *sens_info);
		virtual ~Fitness();

		void compute();

		/// get total_fitness
		double get_total_fitness() { return total_fitness; }
		std::vector<double> get_fitness_details() { return fitness_details; }

		void set_speed_ref(double value);

		FitnessStage* get_stage(int i) { return fitness_tab[i]; }

	protected:
		Ctrl *ctrl;       ///< controller
		SensorsInfo *sens_info; ///< info from the sensor

		std::vector<FitnessStage*> fitness_tab; ///< fitness stages

		double total_fitness; ///< total fitness
		std::vector<double> fitness_details; ///< fitness stages results

		SpeedFitness *speed_fitness;
};

#endif
