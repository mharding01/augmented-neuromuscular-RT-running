/*! 
 * \author Nicolas Van der Noot
 * \file ComputeManager.hh
 * \brief ComputeManager class
 */

#ifndef _COMPUTE_MANAGER_HH_
#define _COMPUTE_MANAGER_HH_

#include <vector> 

#include "Computation.hh"
#include "CtrlInputs.hh"
#include "CtrlOutputs.hh"
#include "CtrlOptions.hh"
#include "MotorCtrlIndex.hh"

enum{KINEMATICS_COMPUTE, HANDLE_INPUTS, WALK_STATES, LOWER_BODY, UPPER_BODY, IMPEDANCE_CTRL, HANDLE_OUTPUTS, MANAGER_NB};

/*! \brief Main manager calling the computation modules
 */
class ComputeManager
{
	public:
		ComputeManager(CtrlInputs *inputs, CtrlOutputs *outputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index);
		~ComputeManager();

		void run_computations();

		inline Computation* get_computation(int index) { return compute_tab[index]; }

	private:
		std::vector<Computation*> compute_tab; ///< computation tabular
		
		CtrlInputs *inputs;   ///< controller inputs
		CtrlOutputs *outputs; ///< controller outputs

		CtrlOptions *options; ///< controller options

		MotorCtrlIndex *ctrl_index; ///< indexes for implemented motors controller
};

#endif
