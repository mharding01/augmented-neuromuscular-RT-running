/*! 
 * \author Nicolas Van der Noot
 * \file StimInit.hh
 * \brief StimInit class
 */

#ifndef _STIM_INIT_HH_
#define _STIM_INIT_HH_

#include "StimulationCtrl.hh"
#include "Articulation.hh"
#include "MainState.hh"

/*! \brief Stimulations to initialize the gait
 */
class StimInit: public StimulationCtrl
{
	public:
		StimInit(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options);
		virtual ~StimInit();

		virtual void compute();

		int is_done() const { return flag_done; }

		// opti parameters
		void set_target_x_com(double value) { target_x_com = value; }
		void set_roll_stim(double value)    { roll_stim    = value; }
		void set_y_com_thres(double value)  { y_com_thres  = value; }

	private:
		Articulation *pitch_hip[NB_LEGS];  ///< pitch hip articulations
		Articulation *pitch_knee[NB_LEGS]; ///< pitch knee articulations
		Articulation *roll_hip[NB_LEGS];   ///< roll hip articulations
		Articulation *roll_foot[NB_LEGS];  ///< roll foot articulations
		Articulation *yaw_hip[NB_LEGS];    ///< yaw hip articulations

		MainState *m_st; ///< main robot states

		double x_com_ref; ///< x reference position for the COM [m]
		double target_x_com; ///< COM x target position [m]

		double roll_stim; ///< stimulation for roll motion [-]
		double y_com_thres; ///< COM y threshold position [m]

		double x_com_foot; ///< x COM position (relative to the first supporting foot) [m]
		double y_com_foot; ///< y COM velocity (relative to the first supporting foot) [m7s]
		double xp_com_foot; ///< x COM position (relative to the first supporting foot) [m]
		double yp_com_foot; ///< y COM velocity (relative to the first supporting foot) [m7s]

		double kp_x_com; ///< proportional gain for x COM position
		double kd_x_com; ///< derivative gain for x COM position

		double kp_pitch_knee; ///< proportional gain for pitch knee angle
		double kp_pitch_hip; ///< proportional gain for pitch hip angle
		double kp_yaw_hip; ///< proportional gain for yaw hip angle

		double t0; ///< initial time [s]
		double t1; ///< final time [s]

		int r_first_swing; ///< 1 if right leg first in swing, 0 otherwise
		int flag_done;     ///< 1 if initiation is done, 0 otherwise

		// function prototypes
		void com_position();
		void com_ref();
};

#endif
