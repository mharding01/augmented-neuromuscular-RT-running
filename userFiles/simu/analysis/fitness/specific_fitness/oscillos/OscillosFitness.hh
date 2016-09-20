/*! 
 * \author Nicolas Van der Noot
 * \file OscillosFitness.hh
 * \brief OscillosFitness class
 */

#ifndef _OSCILLOS_FITNESS_HH_
#define _OSCILLOS_FITNESS_HH_

#include "FitnessStage.hh"
#include "MatsuokaSixN.hh"
#include "Ctrl.hh"

/*! \brief One stage of the fitness function computation:
 * oscillators strike prediction
 */
class OscillosFitness: public FitnessStage
{
	public:
		OscillosFitness(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~OscillosFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		MatsuokaSixN *osc; ///< oscillators
};

#endif
