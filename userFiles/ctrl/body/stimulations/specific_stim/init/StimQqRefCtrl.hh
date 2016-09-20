/*! 
 * \author Nicolas Van der Noot
 * \file StimQqRefCtrl.hh
 * \brief StimQqRefCtrl class
 */

#ifndef _STIM_QQ_REF_CTRL_HH_
#define _STIM_QQ_REF_CTRL_HH_

#include "StimulationCtrl.hh"

/*! \brief stimulations using the CtrlInputs torque references
 */
class StimQqRefCtrl: public StimulationCtrl
{
	public:
		StimQqRefCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimQqRefCtrl();

		virtual void compute();

	private:
		int flag_3D; ///< 1 if 3D walking, 0 otherwise
};

#endif
