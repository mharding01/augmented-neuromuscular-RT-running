/*! 
 * \author Nicolas Van der Noot
 * \file Articulation.hh
 * \brief Articulation class
 */

#ifndef _ARTICULATION_HH_
#define _ARTICULATION_HH_

#include "CtrlInputs.hh"
#include "LowFilter.hh"
#include "CtrlIndex.hh"
#include "MotorCtrlIndex.hh"
#include "body_parts.hh"
#include <cmath>
#include <iostream>
#include <stdlib.h>

#define DEG_TO_RAD (M_PI / 180.0)

// leg articulations
enum{PITCH_FOOT_ART, ROLL_FOOT_ART, PITCH_KNEE_ART, YAW_HIP_ART, ROLL_HIP_ART, PITCH_HIP_ART, LEG_ART_NB};

// torso articulations
enum{PITCH_TORSO_ART, ROLL_TORSO_ART, YAW_TORSO_ART, TORSO_ART_NB};

// arm articulations
enum{PITCH_SHOULDER_ART, ROLL_SHOULDER_ART, YAW_SHOULDER_ART, PITCH_ELBOW_ART, ARM_ART_NB};

/*! \brief Articulation
 */
class Articulation
{
	public:
		Articulation(CtrlInputs *inputs, MotorCtrlIndex *ctrl_index, int art_id, int body_part_id);
		virtual ~Articulation();

		double compute_Qq_soft_contribution(double k_qq);

		virtual void add_Qq_soft_lim() = 0;

		int get_joint_id() const { return joint_id; }

		/// update q and qd values
		void update_q_qd()
		{
			q  = inputs->get_q_mot(joint_id);
			qd = inputs->get_qd_mot(joint_id);
		}

		double get_q()  const { return q; }
		double get_qd() const { return qd; }
		double get_Qq() const { return Qq; }
		double get_Qq_ref() const { return Qq_ref; }

		void reset_Qq() { Qq = 0.0; }
		void add_Qq(double value) { Qq += value; }
		void set_Qq_ref(double value) { Qq_ref = value; }

		void apply_Qq_soft(double k_qq) { add_Qq(filter_Qq_soft->update_and_get(compute_Qq_soft_contribution(k_qq), inputs->get_t())); }

	protected:
		CtrlInputs *inputs; ///< controller inputs

		int body_part_id; ///< body part ID
		int art_id; ///< articulation ID
		int joint_id; ///< joint index (for outputs vector)

		double q;  ///< position [rad]
		double qd; ///< position derivative [rad/s]
		double Qq; ///< torque [Nm]
		double Qq_ref; ///< torque reference (for inverse stimulations) [Nm]

		double q_min; ///< min position value [rad]
		double q_max; ///< max position value [rad]

		LowFilter *filter_Qq_soft; ///< low pass filter

		MotorCtrlIndex *ctrl_index; ///< joint indexes
};

#endif
