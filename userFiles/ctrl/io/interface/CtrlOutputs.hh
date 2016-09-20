/*! 
 * \author Nicolas Van der Noot
 * \file CtrlOutputs.hh
 * \brief CtrlOutputs class
 */

#ifndef _CTRL_OUTPUTS_HH_
#define _CTRL_OUTPUTS_HH_

#include "MotorCtrlIndex.hh"

/*! \brief Outputs of the controller
 */
class CtrlOutputs
{
	public:
		CtrlOutputs(MotorCtrlIndex *ctrl_index);
		~CtrlOutputs();

		// -- get outputs -- //

		double get_q_ref(int mot_id)  { return q_ref[mot_id];   }
		double get_Qq_ref(int mot_id) { return Qq_ref[mot_id];  }


		// -- set outputs -- //

		void set_q_ref(int mot_id, double value)     { q_ref[mot_id]   = value;      }
		void set_Qq_ref(int mot_id, double value)    { Qq_ref[mot_id]  = value;      }

	private:
		int nb_mot; ///< number of motors

		std::vector<double> q_ref;  ///< position reference [rad]
		std::vector<double> Qq_ref; ///< torque reference [Nm]

		MotorCtrlIndex *ctrl_index; ///< indexes for implemented motors controller
};

#endif
