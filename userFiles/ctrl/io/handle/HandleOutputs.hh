/*! 
 * \author Nicolas Van der Noot
 * \file HandleOutputs.hh
 * \brief HandleOutputs class
 */

#ifndef _HANDLE_OUTPUTS_HH_
#define _HANDLE_OUTPUTS_HH_

#include "Computation.hh"
#include "CtrlInputs.hh"
#include "CtrlOutputs.hh"

/*! \brief handles the outputs of the controller
 */
class HandleOutputs: public Computation
{
	public:
		HandleOutputs(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs);
		virtual ~HandleOutputs();

		void compute();

	private:
		CtrlOutputs *outputs; ///< outputs of the controller
};

#endif
