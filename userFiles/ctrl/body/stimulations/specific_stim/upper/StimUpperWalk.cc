#include "StimUpperWalk.hh"
#include "StimWangCtrl.hh"
#include "ctrl_functions.hh"
#include <cmath>

#define SH_PITCH_REF_1 0.3
#define SH_PITCH_REF_2 0.3
#define ELBOW_REF 0.25

inline double pos(double x) { return (x > 0.0) ?  x : 0.0; }
inline double neg(double x) { return (x < 0.0) ? -x : 0.0; }

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 * \param[in] parts body parts
 * \param[in] options controller options
 * \param[in] stim_ctrl lower-body stimulation controller
 */
StimUpperWalk::StimUpperWalk(CtrlInputs *inputs, WalkStates *ws, ForwardKinematics *fwd_kin, BodyPart **parts, CtrlOptions *options, MotorCtrlIndex *ctrl_index, StimulationCtrl *stim_ctrl):
	StimulationCtrl(inputs, ws, fwd_kin, parts, options)
{
	this->stim_ctrl = stim_ctrl;

	ms = static_cast<MainState*>(ws->get_state(MAIN_STATE));

	RightHipPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightHipPitch);
	LeftHipPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftHipPitch);
}

/*! \brief destructor
 */
StimUpperWalk::~StimUpperWalk()
{

}

/*! \brief upper-body main stimulation computations
 */
void StimUpperWalk::compute()
{
	// set stimulations to minimal value
	for(int i=TORSO_BODY; i<=LEFT_ARM_BODY; i++)
	{
		for(int j=0; j<parts[i]->get_nb_muscles(); j++)
		{
			Stim[i][j] = S_MIN;
		}
	}

	// controller according to the COMAN state
	switch (ms->get_coman_state())
	{
		case TEST_STATE:
			break; 
			
		case INIT_UPRIGHT_STATE:
			init();
			break;

		case WALK_COMAN_STATE:
			walk();
			break;
	
		default:
			std::cout << "StimUpperWalk error: not correct COMAN state (" << ms->get_coman_state() << ") !" << std::endl;
			exit(EXIT_FAILURE);
	}

	// limit stimulations
	for(int i=TORSO_BODY; i<=LEFT_ARM_BODY; i++)
	{
		for(int j=0; j<parts[i]->get_nb_muscles(); j++)
		{
			Stim[i][j] = limit_range(Stim[i][j], S_MIN, S_MAX);
		}
	}
}

/*! \brief compute the initial stimulations
 */
void StimUpperWalk::init()
{
	double t;
	double sag_sh_ref, sag_elb_ref;

	// time
	t = inputs->get_t();

	// linear position references evolution
	sag_sh_ref  = linear_interpol(0.0, -SH_PITCH_REF_2, 0.0, 1.0, t);
	sag_elb_ref = linear_interpol(0.0, -ELBOW_REF, 0.0, 1.0, t);

	// shoulder pitch
	p_ctrl(sag_sh_ref, RIGHT_ARM_BODY, PITCH_SHOULDER_ART);
	p_ctrl(sag_sh_ref, LEFT_ARM_BODY,  PITCH_SHOULDER_ART);

	// elbow pitch
	p_ctrl(sag_elb_ref, RIGHT_ARM_BODY, PITCH_ELBOW_ART);
	p_ctrl(sag_elb_ref, LEFT_ARM_BODY,  PITCH_ELBOW_ART);
}

/*! \brief compute the stimulations for the walk
 */
void StimUpperWalk::walk()
{
	double sag_sh_ref;

	// shoulder pitch
	sag_sh_ref = SH_PITCH_REF_1 * (inputs->get_q_mot(LeftHipPitch_id) - inputs->get_q_mot(RightHipPitch_id));

	p_ctrl(sag_sh_ref - SH_PITCH_REF_2, RIGHT_ARM_BODY, PITCH_SHOULDER_ART);
	p_ctrl(-sag_sh_ref - SH_PITCH_REF_2, LEFT_ARM_BODY, PITCH_SHOULDER_ART);

	// elbow pitch
	p_ctrl(-ELBOW_REF, RIGHT_ARM_BODY, PITCH_ELBOW_ART);
	p_ctrl(-ELBOW_REF, LEFT_ARM_BODY,  PITCH_ELBOW_ART);
}

/*! \brief proportional controller
 *
 * \param[in] q_ref position reference [rad]
 * \param[in] body_part_id ID of the body part
 * \param[in] art_id ID of the articulation
 */
void StimUpperWalk::p_ctrl(double q_ref, int body_part_id, int art_id)
{
	double err;

	// error computation
	err = q_ref - parts[body_part_id]->get_q(art_id);

	switch (body_part_id)
	{
		// lower body
		case RIGHT_LEG_BODY :
		case LEFT_LEG_BODY :
			std::cout << "StimUpperWalk error: body part ID (" << body_part_id << ") cannot be from the lower body !" << std::endl;
			exit(EXIT_FAILURE);

		// torso
		case TORSO_BODY :
			switch (art_id)
			{
				case PITCH_TORSO_ART :
					Stim[TORSO_BODY][BET_MUSCLE] = neg(5.0*err);
					Stim[TORSO_BODY][BFL_MUSCLE] = pos(5.0*err);
					break;

				case ROLL_TORSO_ART :
					Stim[TORSO_BODY][BTL_MUSCLE] = neg(5.0*err);
					Stim[TORSO_BODY][BTR_MUSCLE] = pos(5.0*err);
					break;

				case YAW_TORSO_ART :
					Stim[TORSO_BODY][BRL_MUSCLE] = pos(5.0*err);
					Stim[TORSO_BODY][BRR_MUSCLE] = neg(5.0*err);
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
					Stim[RIGHT_ARM_BODY][SET_MUSCLE] = pos(5.0*err);
					Stim[RIGHT_ARM_BODY][SFL_MUSCLE] = neg(5.0*err);
					break;

				case ROLL_SHOULDER_ART :
					Stim[RIGHT_ARM_BODY][SAB_MUSCLE] = neg(5.0*err);
					Stim[RIGHT_ARM_BODY][SAD_MUSCLE] = pos(5.0*err);
					break;

				case YAW_SHOULDER_ART :
					Stim[RIGHT_ARM_BODY][SER_MUSCLE] = neg(5.0*err);
					Stim[RIGHT_ARM_BODY][SIR_MUSCLE] = pos(5.0*err);
					break;

				case PITCH_ELBOW_ART :
					Stim[RIGHT_ARM_BODY][EET_MUSCLE] = pos(5.0*err);
					Stim[RIGHT_ARM_BODY][EFL_MUSCLE] = neg(5.0*err);
					break;

				default:
					std::cout << "StimUpperWalk error: unknown articulation ID (" << art_id << ") !" << std::endl;
					exit(EXIT_FAILURE);
			}
			break;

		// left arm
		case LEFT_ARM_BODY :
			switch (art_id)
			{
				case PITCH_SHOULDER_ART :
					Stim[LEFT_ARM_BODY][SET_MUSCLE] = pos(5.0*err);
					Stim[LEFT_ARM_BODY][SFL_MUSCLE] = neg(5.0*err);
					break;

				case ROLL_SHOULDER_ART :
					Stim[LEFT_ARM_BODY][SAB_MUSCLE] = pos(5.0*err);
					Stim[LEFT_ARM_BODY][SAD_MUSCLE] = neg(5.0*err);
					break;

				case YAW_SHOULDER_ART :
					Stim[LEFT_ARM_BODY][SER_MUSCLE] = pos(5.0*err);
					Stim[LEFT_ARM_BODY][SIR_MUSCLE] = neg(5.0*err);
					break;

				case PITCH_ELBOW_ART :
					Stim[LEFT_ARM_BODY][EET_MUSCLE] = pos(5.0*err);
					Stim[LEFT_ARM_BODY][EFL_MUSCLE] = neg(5.0*err);
					break;

				default:
					std::cout << "StimUpperWalk error: unknown articulation ID (" << art_id << ") !" << std::endl;
					exit(EXIT_FAILURE);
			}
			break;
	
		default:
			std::cout << "StimUpperWalk error: unknown body part ID (" << body_part_id << ") !" << std::endl;
			exit(EXIT_FAILURE);
	}
}
