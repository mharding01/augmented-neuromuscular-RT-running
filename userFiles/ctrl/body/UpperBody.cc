
#include "UpperBody.hh"
#include "ctrl_functions.hh"
#include "CtrlIndex.hh"
#include <cmath>

#define SH_ROLL_REF 0.09
#define SH_PITCH_REF_1 0.3
#define SH_PITCH_REF_2 0.3
#define SH_YAW_REF 0.14
#define ELBOW_REF 0.25
#define ELBOW_HOME (3.0*DEG_TO_RAD)

#define DEG_TO_RAD (M_PI / 180.0)

#define FIXED_SH_LAT_REF ( M_PI / 6.0)
#define FIXED_ELBOW_REF (- M_PI / 2.0)

#define T0 0.0
#define T1 0.5

/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[out] outputs controller outputs
 * \param[in] ws walk states
 */
UpperBody::UpperBody(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, WalkStates *ws):
	Computation(inputs, options, ctrl_index)
{
	this->outputs = outputs;
	this->ws = ws;

	ms = static_cast<MainState*>(ws->get_state(MAIN_STATE));

	RightShPitch_id  = ctrl_index->get_inv_index(CtrlIndex::RightShPitch);
	RightShRoll_id   = ctrl_index->get_inv_index(CtrlIndex::RightShRoll);
	RightShYaw_id    = ctrl_index->get_inv_index(CtrlIndex::RightShYaw);
	RightElbPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightElbPitch);
	RightHipPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightHipPitch);

	LeftShPitch_id  = ctrl_index->get_inv_index(CtrlIndex::LeftShPitch);
	LeftShRoll_id   = ctrl_index->get_inv_index(CtrlIndex::LeftShRoll);
	LeftShYaw_id    = ctrl_index->get_inv_index(CtrlIndex::LeftShYaw);
	LeftElbPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftElbPitch);
	LeftHipPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftHipPitch);
}

/*! \brief destructor
 */
UpperBody::~UpperBody()
{

}

/*! \brief main computation
 */
void UpperBody::compute()
{
	double t;
	double sag_sh_ref_a, sag_sh_ref_b, elbow_ref;

	t = inputs->get_t();

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

	if (options->is_upper_motion())
	{
		// shoulder lateral
		outputs->set_q_ref(RightShRoll_id, -SH_ROLL_REF);
		outputs->set_q_ref(LeftShRoll_id,  SH_ROLL_REF);

		// shoulder sagittal
		outputs->set_q_ref(RightShPitch_id,  sag_sh_ref_a - sag_sh_ref_b);
		outputs->set_q_ref(LeftShPitch_id, -sag_sh_ref_a - sag_sh_ref_b);

		// shoulder transverse
		outputs->set_q_ref(RightShYaw_id, -SH_YAW_REF);
		outputs->set_q_ref(LeftShYaw_id,  SH_YAW_REF);

		// pitch elbow
		outputs->set_q_ref(RightElbPitch_id, -elbow_ref);
		outputs->set_q_ref(LeftElbPitch_id, -elbow_ref);
	}
	else
	{
		// shoulder lateral
		outputs->set_q_ref(RightShRoll_id, -FIXED_SH_LAT_REF);
		outputs->set_q_ref(LeftShRoll_id,  FIXED_SH_LAT_REF);

		// shoulder sagittal
		outputs->set_q_ref(RightShPitch_id, 0.0);
		outputs->set_q_ref(LeftShPitch_id, 0.0);

		// shoulder transverse
		outputs->set_q_ref(RightShYaw_id, 0.0);
		outputs->set_q_ref(LeftShYaw_id, 0.0);

		// pitch elbow
		outputs->set_q_ref(RightElbPitch_id, FIXED_ELBOW_REF);
		outputs->set_q_ref(LeftElbPitch_id, FIXED_ELBOW_REF);
	}
}
