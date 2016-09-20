/*! 
 * \author Nicolas Van der Noot
 * \file Computation.hh
 * \brief Computation class
 */

#ifndef _COMPUTATION_HH_
#define _COMPUTATION_HH_

#include "CtrlInputs.hh"
#include "CtrlOptions.hh"
#include "MotorCtrlIndex.hh"

/*! \brief Computation parent
 */
class Computation
{
	public:
		Computation(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index);
		virtual ~Computation();

		virtual void compute() = 0;

	protected:
		CtrlInputs *inputs;      ///< controller inputs
		CtrlOptions *options;    ///< controller options
		MotorCtrlIndex *ctrl_index; ///< indexes of the motors
};

#endif
