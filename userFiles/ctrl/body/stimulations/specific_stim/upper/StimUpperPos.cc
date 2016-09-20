
#include "StimUpperPos.hh"
#include "ctrl_functions.hh"
#include "CtrlIndex.hh"
#include <cmath>

#define SH_ROLL_REF 0.09
#define SH_PITCH_REF_1 0.3
#define SH_PITCH_REF_2 0.3
#define SH_YAW_REF 0.14
#define ELBOW_REF 0.25
#define ELBOW_HOME (3.0*DEG_TO_RAD)

#define FIXED_SH_LAT_REF ( M_PI / 6.0)
#define FIXED_ELBOW_REF (- M_PI / 2.0)

#define T0 0.0
#define T1 0.5

inline double pos(double x) { return (x > 0.0) ?  x : 0.0; }
inline double neg(double x) { return (x < 0.0) ? -x : 0.0; }

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 */
StimUpperPos::StimUpperPos(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options, MotorCtrlIndex *ctrl_index): 
	StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	this->ctrl_index = ctrl_index;

	RightHipPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightHipPitch);
	LeftHipPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftHipPitch);
}

/*! \brief destructor
 */
StimUpperPos::~StimUpperPos()
{

}

/*! \brief stimulations main computation
 */
void StimUpperPos::compute()
{
	double t;
	double sag_sh_ref_a, sag_sh_ref_b, elbow_ref;

	// time
	t = inputs->get_t();

	// position references
	sag_sh_ref_a = SH_PITCH_REF_1 * (inputs->get_q_mot(LeftHipPitch_id) - inputs->get_q_mot(RightHipPitch_id));

	if (t < 0.5)
	{
		sag_sh_ref_b = linear_interpol(0.0, SH_PITCH_REF_2, T0, T1, t);
		elbow_ref = linear_interpol(ELBOW_HOME, ELBOW_REF , T0, T1, t);
	}
	else
	{
		sag_sh_ref_b = SH_PITCH_REF_2;
		elbow_ref = ELBOW_REF;
	}

	// torso
	p_ctrl(0.0, 0.0, TORSO_BODY, PITCH_TORSO_ART);
	p_ctrl(0.0, 0.0, TORSO_BODY, ROLL_TORSO_ART);
	p_ctrl(0.0, 0.0, TORSO_BODY, YAW_TORSO_ART);

	if (options->is_upper_motion()) // upper motion
	{
		// shoulder roll
		p_ctrl(-SH_ROLL_REF, 0.0, RIGHT_ARM_BODY, ROLL_SHOULDER_ART);
		p_ctrl(SH_ROLL_REF , 0.0, LEFT_ARM_BODY,  ROLL_SHOULDER_ART);

		// shoulder pitch
		p_ctrl( sag_sh_ref_a - sag_sh_ref_b, 0.0, RIGHT_ARM_BODY, PITCH_SHOULDER_ART);
		p_ctrl(-sag_sh_ref_a - sag_sh_ref_b, 0.0, LEFT_ARM_BODY,  PITCH_SHOULDER_ART);

		// shoulder yaw
		p_ctrl(-SH_YAW_REF, 0.0, RIGHT_ARM_BODY, YAW_SHOULDER_ART);
		p_ctrl( SH_YAW_REF, 0.0, LEFT_ARM_BODY,  YAW_SHOULDER_ART);

		// elbow pitch
		p_ctrl(-elbow_ref, 0.0, RIGHT_ARM_BODY, PITCH_ELBOW_ART);
		p_ctrl(-elbow_ref, 0.0, LEFT_ARM_BODY,  PITCH_ELBOW_ART);
	}
	else // no upper motion
	{
		// shoulder roll
		p_ctrl(-FIXED_SH_LAT_REF, 0.0, RIGHT_ARM_BODY, ROLL_SHOULDER_ART);
		p_ctrl(FIXED_SH_LAT_REF, 0.0, LEFT_ARM_BODY,  ROLL_SHOULDER_ART);

		// shoulder pitch
		p_ctrl(0.0, 0.0, RIGHT_ARM_BODY, PITCH_SHOULDER_ART);
		p_ctrl(0.0, 0.0, LEFT_ARM_BODY,  PITCH_SHOULDER_ART);

		// shoulder yaw
		p_ctrl(0.0, 0.0, RIGHT_ARM_BODY, YAW_SHOULDER_ART);
		p_ctrl(0.0, 0.0, LEFT_ARM_BODY,  YAW_SHOULDER_ART);

		// elbow pitch
		p_ctrl(FIXED_ELBOW_REF, 0.0, RIGHT_ARM_BODY, PITCH_ELBOW_ART);
		p_ctrl(FIXED_ELBOW_REF, 0.0, LEFT_ARM_BODY,  PITCH_ELBOW_ART);
	}
}

/*! \brief proportional controller
 *
 * \param[in] q_ref position reference [rad]
 * \param[in] q_ref_d derovative of 'q_ref' [rad/s]
 * \param[in] body_part_id ID of the body part
 * \param[in] art_id ID of the articulation
 */
void StimUpperPos::p_ctrl(double q_ref, double q_ref_d, int body_part_id, int art_id)
{
	double err, err_d;

	// error computation
	err   = q_ref   - parts[body_part_id]->get_q(art_id);
	err_d = q_ref_d - parts[body_part_id]->get_qd(art_id);

	switch (body_part_id)
	{
		// lower body
		case RIGHT_LEG_BODY :
		case LEFT_LEG_BODY :
			std::cout << "StimUpperPos error: body part ID (" << body_part_id << ") cannot be from the lower body !" << std::endl;
			exit(EXIT_FAILURE);

		// torso
		case TORSO_BODY :
			switch (art_id)
			{
				case PITCH_TORSO_ART :
					Stim[TORSO_BODY][BET_MUSCLE] = neg(2.5*err);
					Stim[TORSO_BODY][BFL_MUSCLE] = pos(2.5*err);
					break;

				case ROLL_TORSO_ART :
					Stim[TORSO_BODY][BTL_MUSCLE] = neg(10.0*err);
					Stim[TORSO_BODY][BTR_MUSCLE] = pos(10.0*err);
					break;

				case YAW_TORSO_ART :
					Stim[TORSO_BODY][BRL_MUSCLE] = pos(10.0*err);
					Stim[TORSO_BODY][BRR_MUSCLE] = neg(10.0*err);
					break;
			
				default:
					std::cout << "StimUpperPos error: unknown articulation ID (" << art_id << ") !" << std::endl;
					exit(EXIT_FAILURE);
			}
			break;

		// right arm
		case RIGHT_ARM_BODY :
			switch (art_id)
			{
				case PITCH_SHOULDER_ART :
					Stim[RIGHT_ARM_BODY][SET_MUSCLE] = pos(10.0*err);
					Stim[RIGHT_ARM_BODY][SFL_MUSCLE] = neg(10.0*err);
					break;

				case ROLL_SHOULDER_ART :
					Stim[RIGHT_ARM_BODY][SAB_MUSCLE] = neg(10.0*err);
					Stim[RIGHT_ARM_BODY][SAD_MUSCLE] = pos(10.0*err);
					break;

				case YAW_SHOULDER_ART :
					Stim[RIGHT_ARM_BODY][SER_MUSCLE] = neg(10.0*err);
					Stim[RIGHT_ARM_BODY][SIR_MUSCLE] = pos(10.0*err);
					break;

				case PITCH_ELBOW_ART :
					Stim[RIGHT_ARM_BODY][EET_MUSCLE] = pos(10.0*err);
					Stim[RIGHT_ARM_BODY][EFL_MUSCLE] = neg(10.0*err);
					break;

				default:
					std::cout << "StimUpperPos error: unknown articulation ID (" << art_id << ") !" << std::endl;
					exit(EXIT_FAILURE);
			}
			break;

		// left arm
		case LEFT_ARM_BODY :
			switch (art_id)
			{
				case PITCH_SHOULDER_ART :
					Stim[LEFT_ARM_BODY][SET_MUSCLE] = pos(10.0*err);
					Stim[LEFT_ARM_BODY][SFL_MUSCLE] = neg(10.0*err);
					break;

				case ROLL_SHOULDER_ART :
					Stim[LEFT_ARM_BODY][SAB_MUSCLE] = pos(10.0*err);
					Stim[LEFT_ARM_BODY][SAD_MUSCLE] = neg(10.0*err);
					break;

				case YAW_SHOULDER_ART :
					Stim[LEFT_ARM_BODY][SER_MUSCLE] = pos(10.0*err);
					Stim[LEFT_ARM_BODY][SIR_MUSCLE] = neg(10.0*err);
					break;

				case PITCH_ELBOW_ART :
					Stim[LEFT_ARM_BODY][EET_MUSCLE] = pos(10.0*err);
					Stim[LEFT_ARM_BODY][EFL_MUSCLE] = neg(10.0*err);
					break;

				default:
					std::cout << "StimUpperPos error: unknown articulation ID (" << art_id << ") !" << std::endl;
					exit(EXIT_FAILURE);
			}
			break;
	
		default:
			std::cout << "StimUpperPos error: unknown body part ID (" << body_part_id << ") !" << std::endl;
			exit(EXIT_FAILURE);
	}
}