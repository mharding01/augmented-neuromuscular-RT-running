/*! 
 * \author Nicolas Van der Noot
 * \file CtrlOptions.hh
 * \brief CtrlOptions class
 */

#ifndef _CTRL_OPTIONS_HH_
#define _CTRL_OPTIONS_HH_

#ifdef ROBOTRAN_SIMU
	#include "user_realtime.h"
#endif

enum{CTRL_DEFAULT, CTRL_REFLEX, CTRL_CPG, CTRL_STIM_TEST, CTRL_REFLEX_WANG};
enum{COMAN_DEFAULT, COMAN_SHORT_FEET, COMAN_FLEX_CAT_1, COMAN_FLEX_CAT_2};

/*! \brief controller options
 */
class CtrlOptions
{
	public:
		CtrlOptions();
		~CtrlOptions();

		int get_flag_coman() const { return flag_coman; }

		int get_flag_ctrl() const { return flag_ctrl; }

		int is_flag_3D() const { return flag_3D; }

		int is_r_first_swing() const { return r_first_swing; }

		int is_upper_motion() const { return upper_motion; }

		int is_torque_limit() const { return torque_limit; }

		int is_extra_knee() const { return extra_knee; }

		int is_cpg_range() const { return cpg_range; }

		int is_extra_ham_reflex() const { return extra_ham_reflex; }

		int is_extra_ta_reflex() const { return extra_ta_reflex; }

		int is_opti() const { return opti; }

		int is_muscle_noise() const { return muscle_noise; }

		int is_print() const { return print; }

		int is_ctrl_two_parts() const { return ctrl_two_parts; }

		int is_Qq_match_wang() const { return flag_Qq_match_wang; }

		int is_initial_pos() const { return flag_initial_pos; }

		int is_apply_Qq_wang() const { return apply_Qq_wang; }

	private:
		int flag_coman;     ///< flag corresponding to the COMAN model used
		int flag_ctrl;      ///< flag corresponding to the controller type
		int flag_3D;        ///< 1 if 3D walking, 0 otherwise
		int r_first_swing;  ///< 1 if right leg first in swing, 0 otherwise
		int upper_motion;   ///< 1 for upper-body motion, 0 otherwise
		int torque_limit;   ///< 1 to limit the torque outputs, 0 otherwise
		int extra_knee;     ///< 1 for the extra swing knee CPG stimulation
		int cpg_range;      ///< 1 for CPG range speed, 0 for fixed CPG speed
		int extra_ham_reflex; ///< extra reflex to flex the knee in early swing
		int extra_ta_reflex;  ///< extra reflex to flex the foot in late swing
		int opti;             ///< 1 to run an optimization, 0 otherwise
		int muscle_noise;     ///< 1 to add noise on the muscle stimulations, 0 otherwise
		int print;            ///< 1 to print comments, 0 otherwise
		int ctrl_two_parts;   ///< 1 for ctrl in two parts : first with know results and after with opti's results, 0 otherwise
		int flag_Qq_match_wang; ///< 1 to find param from Qq wang with driven joints for position, 0 otherwise
		int flag_initial_pos; ///< 1 to optimize initial positions and velocities for joints, 0 otherwise
		int apply_Qq_wang;  ///< 1 to apply Wang torque (with opti shifting and sclaling) in leg pitch articulations, 0 otherwise
};

#endif
