/*! 
 * \author Nicolas Van der Noot
 * \file StimMin.hh
 * \brief StimMin class
 */
#ifndef _STIM_MIN_HH_
#define _STIM_MIN_HH_

#include "StimulationCtrl.hh"

/*! \brief controller to send S_MIN to all muscles
 */
class StimMin: public StimulationCtrl
{
	public:
		StimMin(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimMin();

		virtual void compute();
};

#endif
