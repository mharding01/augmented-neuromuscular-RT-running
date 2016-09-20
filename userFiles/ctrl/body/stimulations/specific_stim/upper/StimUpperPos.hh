/*! 
 * \author Nicolas Van der Noot
 * \file StimUpperPos.hh
 * \brief StimUpperPos class
 */

#ifndef _STIM_UPPER_HH_
#define _STIM_UPPER_HH_

#include "StimulationCtrl.hh"
#include "MotorCtrlIndex.hh"

/*! \brief stimulations sent to all the upper muscles
 */
class StimUpperPos: public StimulationCtrl
{
	public:
		StimUpperPos(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options, MotorCtrlIndex *ctrl_index);
		virtual ~StimUpperPos();

		virtual void compute();

	private:
		MotorCtrlIndex *ctrl_index; ///< indexes of the motors

		int RightHipPitch_id; ///< right hip pitch joint
		int LeftHipPitch_id;  ///< left hip pitch joint

		void p_ctrl(double q_ref, double q_ref_d, int body_part_id, int art_id);
};

#endif
