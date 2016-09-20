/*! 
 * \author Nicolas Van der Noot
 * \file StimulationCtrl.hh
 * \brief StimulationCtrl class
 */

#ifndef _STIMULATION_CTRL_HH_
#define _STIMULATION_CTRL_HH_

#include "CtrlInputs.hh"
#include "BodyPart.hh"
#include "WalkStates.hh"
#include "CtrlOptions.hh"
#include "coman_properties.hh"
#include "ForwardKinematics.hh"

#include <vector>
#include <stdlib.h>

/*! \brief Controller generating the stimulations sent to the muscles
 */
class StimulationCtrl
{
	public:
		StimulationCtrl(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimulationCtrl();

		virtual void compute() = 0;

		double get_Stim(int body_part_id, int muscle_id) { return Stim[body_part_id][muscle_id]; }
		
	protected:
		std::vector<double> Stim[NB_BODY_PARTS]; ///< stimulations

		CtrlInputs *inputs; ///< controller inputs
		BodyPart **parts;   ///< body parts
		WalkStates *ws;     ///< walk states
		ForwardKinematics *fwd_kin; ///< forward kinematics

		CtrlOptions *options; ///< controller options
};

#endif
