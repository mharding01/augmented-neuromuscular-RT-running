/*! 
 * \author Nicolas Van der Noot
 * \file StimUpperWalk.hh
 * \brief StimUpperWalk class
 */

#ifndef _STIM_UPPER_WALK_HH_
#define _STIM_UPPER_WALK_HH_

#include "StimulationCtrl.hh"
#include "MainState.hh"

/*! \brief stimulation for upper-body with StimWalkCtrl
 */
class StimUpperWalk: public StimulationCtrl
{
	public:
		StimUpperWalk(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options, MotorCtrlIndex *ctrl_index, StimulationCtrl *stim_ctrl);
		virtual ~StimUpperWalk();

		virtual void compute();

	private:
		StimulationCtrl *stim_ctrl; ///< stimulation controller
		MainState *ms; ///< main state

		int RightHipPitch_id; ///< right hip pitch joint
		int LeftHipPitch_id;  ///< left hip pitch joint

		// function prototypes
		void init();
		void walk();
		void p_ctrl(double q_ref, int body_part_id, int art_id);
};

#endif
