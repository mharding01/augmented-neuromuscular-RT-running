/*! 
 * \author Nicolas Van der Noot
 * \file DSFitness.hh
 * \brief DSFitness class
 */

#ifndef _DS_FITNESS_HH_
#define _DS_FITNESS_HH_

#include "FitnessStage.hh"
#include "SwingStanceState.hh"
#include "Ctrl.hh"

/*! \brief One stage of the fitness function computation:
 * minimise double support time per cycle
 */
class DSFitness: public FitnessStage
{
	public:
		DSFitness(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~DSFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		SwingStanceState *sw_st; ///< swing-stance state

		double tf; ///< simulation final time [s]
		double dt; ///< time step [s]
		double time_ds; ///< time of double support [s]
		double ds_per_cycle; ///< mean time of double support
		int ds_opti_started; ///< 1 if computation started, 0 otherwise
		double ds_opti_t_start; ///< time when computation started [s]
};

#endif
