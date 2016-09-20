/*! 
 * \author Nicolas Van der Noot
 * \file MetEnergyFitness.hh
 * \brief MetEnergyFitness class
 */

#ifndef _MET_ENERGY_FITNESS_HH_
#define _MET_ENERGY_FITNESS_HH_

#include "FitnessStage.hh"
#include "MinDistFitness.hh"
#include "Ctrl.hh"
#include "Body.hh"
#include "SimuOptions.h"

/*! \brief One stage of the fitness function computation:
 * minimizing virtual muscles metabolic energy consumption
 */
class MetEnergyFitness: public FitnessStage
{
	public:
		MetEnergyFitness(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~MetEnergyFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		MinDistFitness *min_dist_fit; ///< minimal distance fitness stage

		Body *lower_body; ///< lower body

		SimuOptions *options; ///< simulation options

		int met_energy_init_flag; ///< 1 if started to compute, 0 otherwise

		double met_energy_init; ///< energy at initialization [J]
		double met_energy_end;  ///< energy at end [J]

		double met_energy_dist; ///< energy per unit distance

		double dt; 
		
		int printed;
};

#endif
