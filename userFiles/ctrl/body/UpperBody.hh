/*! 
 * \author Nicolas Van der Noot
 * \file UpperBody.hh
 * \brief UpperBody class
 */

#ifndef _UPPER_BODY_HH_
#define _UPPER_BODY_HH_

#include "Computation.hh"

#include "CtrlInputs.hh"
#include "CtrlOutputs.hh"
#include "WalkStates.hh"
#include "MainState.hh"

/*! \brief Upper body controller
 */
class UpperBody: public Computation
{
	public:
		UpperBody(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, WalkStates *ws);
		virtual ~UpperBody();

		virtual void compute();

	private:
		CtrlOutputs *outputs; ///< controller outputs
		WalkStates *ws;       ///< walk states
		MainState *ms;        ///< main COMAN state

		// i/o vector IDs
		int RightShPitch_id;  ///< right shoulder pitch joint
		int RightShRoll_id;   ///< right shoulder roll joint
		int RightShYaw_id;    ///< right shoulder yaw joint
		int RightElbPitch_id; ///< right elbow pitch joint
		int RightHipPitch_id; ///< right hip pitch joint

		int LeftShPitch_id;   ///< left shoulder pitch joint
		int LeftShRoll_id;    ///< left shoulder roll joint
		int LeftShYaw_id;     ///< left shoulder yaw joint
		int LeftElbPitch_id;  ///< left elbow pitch joint
		int LeftHipPitch_id;  ///< left hip pitch joint
};

#endif
