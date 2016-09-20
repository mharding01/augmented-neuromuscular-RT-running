/*! 
 * \author Nicolas Van der Noot
 * \file HandleInputs.hh
 * \brief HandleInputs class
 */
#ifndef _HANDLE_INPUTS_HH_
#define _HANDLE_INPUTS_HH_

#include "CtrlInputs.hh"
#include "Computation.hh"
#include "RunningAverage.hh"
#include "ForwardKinematics.hh"

// anchor point position [m]
#define D2_1 0.020282
#define D2_3 0.119121
#define D4_3 0.047500
#define D5_1 -0.015000
#define D5_2 -0.082500
#define D5_3 0.155400
#define D6_2 -0.071000
#define D7_3 -0.042500
#define D8_3 -0.137500
#define D9_1 -0.015000
#define D9_2 0.082500
#define D9_3 0.155400
#define D10_2 0.071000
#define D11_3 -0.042500
#define D12_3 -0.137500
#define D13_2 -0.023000
#define D14_2 -0.049600
#define D15_3 -0.102400
#define D16_3 -0.123400
#define D17_3 -0.201000
#define D19_2 0.023000
#define D20_2 0.049600
#define D21_3 -0.102400
#define D22_3 -0.123400
#define D23_3 -0.201000

/*! \brief Handle the inputs
 */
class HandleInputs: public Computation
{
	public:
		HandleInputs(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, ForwardKinematics *fwd_kin);
		virtual ~HandleInputs();

		virtual void compute();

	private:
		ForwardKinematics *fwd_kin; ///< foward kinematics

		RunningAverage *feet_av_compute[NB_LEGS]; ///< computation class for the average of feet forces [N]
		RunningAverage *toe_av_compute[NB_LEGS]; ///< computation class for the average of toes forces [N]

		void feet_forces();
		void toe_forces();
};

#endif

