/*! 
 * \author Nicolas Van der Noot
 * \file StimTrapezoid.hh
 * \brief StimTrapezoid class
 */
#ifndef _STIM_TRAPEZOID_HH_
#define _STIM_TRAPEZOID_HH_

#include "StimulationCtrl.hh"

/*! \brief controller to send trapezoidal stimulations to muscles
 */
class StimTrapezoid: public StimulationCtrl
{
	public:
		StimTrapezoid(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimTrapezoid();

		virtual void compute();

	private:
		double time_A; ///< first special time [s]
		double time_B; ///< second special time [s]
		double time_C; ///< third special time [s]

		// function prototypes
		double trap_cur_value(double cur_t, double init_t, double time_A, double time_B, double time_C);
		void stim_member(int part_id, double init_t, double delta_t);
		void stim_all(double init_t, double delta_t);
};

#endif
