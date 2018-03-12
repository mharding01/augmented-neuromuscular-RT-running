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

		/// get met_energy_total_norm
		double get_met_energy_total_norm() const { return met_energy_total_norm; }

		/// get met_energy_legs_norm
		double get_met_energy_legs_norm() const { return met_energy_legs_norm; }

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

		// for data extraction
		double met_energy_total_init; ///< total metabolic energy (init)
		double met_energy_legs_init;  ///< energy for the legs (init)

		double met_energy_total_end; ///< total metabolic energy (end)
		double met_energy_legs_end;  ///< energy for the legs (end)

		double dist_init; ///< initial distance [m]
		double dist_end;  ///< final distance [m]

		bool min_energy_flag; ///< true if enrgy init done, false otherwise
		bool max_energy_flag; ///< true if enrgy end done, false otherwise

		double met_energy_total_norm; ///< 'met_energy_total' normalized to walking distance and robot mass
		double met_energy_legs_norm;  ///< 'met_energy_legs' normalized to walking distance and robot mass
};

#endif
