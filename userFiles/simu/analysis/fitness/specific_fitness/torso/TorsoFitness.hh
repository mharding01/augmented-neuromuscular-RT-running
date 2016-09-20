/*! 
 * \author Nicolas Van der Noot
 * \file TorsoFitness.hh
 * \brief TorsoFitness class
 */

#ifndef _TORSO_FITNESS_HH_
#define _TORSO_FITNESS_HH_

#include "FitnessStage.hh"
#include "SensorsInfo.hh"
#include "Ctrl.hh"
#include "CtrlInputs.hh"
#include "AverageIncrement.hh"

/*! \brief One stage of the fitness function computation:
 * velocity target matching
 */
class TorsoFitness: public FitnessStage
{
	public:
		TorsoFitness(MbsData *mbs_data, SensorsInfo *sens_info, Ctrl *ctrl);
		virtual ~TorsoFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		SensorsInfo *sens_info; ///< info from the sensors
		Ctrl *ctrl;
		CtrlInputs *inputs;

		AverageIncrement *torso_av;

		double torso_mean;
		double opti_started;
};

#endif
