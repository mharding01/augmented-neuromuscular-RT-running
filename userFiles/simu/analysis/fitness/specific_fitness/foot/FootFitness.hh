/*! 
 * \author Nicolas Van der Noot
 * \file FootFitness.hh
 * \brief FootFitness class
 */

#ifndef _FOOT_FITNESS_HH_
#define _FOOT_FITNESS_HH_

#include "FitnessStage.hh"
#include "SensorsInfo.hh"
#include "SwingStanceState.hh"
#include "Ctrl.hh"

/*! \brief One stage of the fitness function computation:
 * walk a minimal distance
 */
class FootFitness: public FitnessStage
{
	public:
		FootFitness(MbsData *mbs_data, SensorsInfo *sens_info, Ctrl *ctrl);
		virtual ~FootFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		SensorsInfo *sens_info; ///< sensors info
		SwingStanceState *sw_st;

		double dt; 

		int foot_opti_started;
		double foot_opti_t_start;

		double stance_r;
		double stance_l;
		double stance_mean;
};

#endif
