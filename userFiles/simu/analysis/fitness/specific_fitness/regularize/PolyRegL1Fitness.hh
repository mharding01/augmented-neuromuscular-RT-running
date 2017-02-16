/*! 
 * \author Matthew Harding
 * \file PolyRegL1Fitness.hh
 * \brief PolyRegL1Fitness class, used to regularize polynomial fitting
 * as a Gaussian fitness function according to L1-norm
 */

#ifndef _POLYREGL1_FITNESS_HH
#define _POLYREGL1_FITNESS_HH

#include "FitnessStage.hh"
#include "MatsuokaSixN.hh"
#include "Ctrl.hh"

/*! \brief One stage of the fitness function computation:
 * polynomial fit parameter regularization - L1
 */
class PolyRegL1Fitness: public FitnessStage
{
	public:
		PolyRegL1Fitness(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~PolyRegL1Fitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		MatsuokaSixN *osc; ///< oscillators
        double l1_norm; ///< saves sum of magnitudes of poly params, L1-norm
        double fitness; ///< computed on construction
};

#endif
