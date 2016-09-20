/*! 
 * \author Nicolas Van der Noot
 * \file TorqueFitness.hh
 * \brief TorqueFitness class
 */

#ifndef _TORQUE_FITNESS_HH_
#define _TORQUE_FITNESS_HH_

#include "FitnessStage.hh"
#include "CtrlOutputs.hh"
#include "Ctrl.hh"

/*! \brief One stage of the fitness function computation:
 * walk a minimal distance
 */
class TorqueFitness: public FitnessStage
{
	public:
		TorqueFitness(MbsData *mbs_data, Ctrl *ctrl);
		virtual ~TorqueFitness();

		virtual void compute();
		virtual double get_fitness();
		virtual int next_stage_unlocked();

	private:
		CtrlOutputs *outputs;    ///< controller outputs

		int RightHipPitch_id;
		int RightKneePitch_id;
		int RightFootPitch_id;

		double dt;

		std::vector<double> RHipQq;
		std::vector<double> RKneeQq;
		std::vector<double> RAnkleQq;

		double serror;

		double l_coman; ///< coman leg length
		double l_wang; ///< wang humanoid leg length
};

#endif
