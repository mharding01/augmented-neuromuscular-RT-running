/*! 
 * \author Nicolas Van der Noot
 * \file JointLimit.hh
 * \brief JointLimit class
 */

#ifndef _JOINT_LIMIT_HH_
#define _JOINT_LIMIT_HH_

#include "mbs_data.h"
#include "ModelSimuIndex.hh"
#include "SimuOptions.h"

/*! \brief joint limit (range)
 */
class JointLimit
{
	public:
		JointLimit(MbsData *mbs_data, ModelSimuIndex *simu_index, int joint_id, double q_min, double q_max);
		~JointLimit();

		void check();

	private:
		MbsData *mbs_data; ///< Robotran structure

		ModelSimuIndex *simu_index; ///< simulation indexes list

		SimuOptions *options; ///< simulation options

		double q_min; ///< minimal joint range value [rad]
		double q_max; ///< maximal joint range value [rad]

		int joint_id; ///< simulation joint ID
		int mbs_id;   ///< MBS file joint ID

		double q; ///< joint position [rad]
};

#endif
