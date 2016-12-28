
#include "JointsInit.hh"
#include "user_all_id.h"
#include "cmake_define.h"

#include "CtrlIndex.hh"

#define SH_PITCH_REF_1 0.3
#define SH_PITCH_REF_2 0.3
#define ELBOW_REF 0.25

/*! \brief constructor
 */
JointsInit::JointsInit(MbsData *mbs_data, CtrlInputs *inputs)
{
	this->mbs_data = mbs_data;
	this->inputs = inputs;

	T1_p = CMAKE_VAR_2;
	T3   = 0.51434532;
	T3_p = -0.19917298;
	R2   = 0.15931929;
	R2_p = 0.12469546;

	r_sh   = 0.0;
	r_sh_p = 1.78067287;

	r_hip     = -0.53664085;
	r_hip_p   = 2.35038869;
	r_knee    = 0.50076082;
	r_knee_p  = 1.65588274;
	r_ankle   = 0.04930992;
	r_ankle_p = 4.63256804;

	l_hip     = -0.01129314;
	l_hip_p   = -1.50775015;
	l_knee    = 0.96870977;
	l_knee_p  = -0.15919546;
	l_ankle   = 0.27641770;
	l_ankle_p = -1.33132669;

	nb_mot = inputs->get_nb_mot();

	inputs->get_opti_inputs()->set_joints_init(this);
}

/*! \brief destructor
 */
JointsInit::~JointsInit()
{

}

/*! \brief set init pos
 */
void JointsInit::set_joints_init()
{
	double sag_sh_ref;
	sag_sh_ref = SH_PITCH_REF_1*(l_hip-r_hip);

	mbs_data->qd[FJ_T1_Coman_id] = T1_p;
	mbs_data->q[FJ_T3_Coman_id]  = T3;
	mbs_data->qd[FJ_T3_Coman_id] = T3_p;
	mbs_data->q[FJ_R2_Coman_id]  = R2;
	mbs_data->qd[FJ_R2_Coman_id] = R2_p;

	mbs_data->q[RShSag_id] = sag_sh_ref - SH_PITCH_REF_2;
	mbs_data->qd[RShSag_id] = r_sh_p;
	mbs_data->q[LShSag_id] = -sag_sh_ref - SH_PITCH_REF_2;
	mbs_data->qd[LShSag_id] = -r_sh_p;
	mbs_data->q[RElb_id] = -ELBOW_REF;
	mbs_data->q[LElb_id] = -ELBOW_REF;

	mbs_data->q[RHipSag_id]   = r_hip;
	mbs_data->qd[RHipSag_id]  = r_hip_p;
	mbs_data->q[RKneeSag_id]  = r_knee;
	mbs_data->qd[RKneeSag_id] = r_knee_p;
	mbs_data->q[RAnkSag_id]   = r_ankle;
	mbs_data->qd[RAnkSag_id]  = r_ankle_p;

	mbs_data->q[LHipSag_id]   = l_hip;
	mbs_data->qd[LHipSag_id]  = l_hip_p;
	mbs_data->q[LKneeSag_id]  = l_knee;
	mbs_data->qd[LKneeSag_id] = l_knee_p;
	mbs_data->q[LAnkSag_id]   = l_ankle;
	mbs_data->qd[LAnkSag_id]  = l_ankle_p;

	// initial motors position and velocity (before the spring)
	mbs_data->ux[CtrlIndex::RightShPitch+1] = sag_sh_ref - SH_PITCH_REF_2;
	mbs_data->ux[CtrlIndex::RightShPitch+nb_mot+1] = r_sh_p;
	mbs_data->ux[CtrlIndex::LeftShPitch+1] = -sag_sh_ref - SH_PITCH_REF_2;
	mbs_data->ux[CtrlIndex::LeftShPitch+nb_mot+1] = -r_sh_p;
	mbs_data->ux[CtrlIndex::RightElbPitch+1] = -ELBOW_REF;
	mbs_data->ux[CtrlIndex::LeftElbPitch+1] = -ELBOW_REF;

	mbs_data->ux[CtrlIndex::RightHipPitch+1]   = r_hip;
	mbs_data->ux[CtrlIndex::RightHipPitch+nb_mot+1]  = r_hip_p;
	mbs_data->ux[CtrlIndex::RightKneePitch+1]  = r_knee;
	mbs_data->ux[CtrlIndex::RightKneePitch+nb_mot+1] = r_knee_p;
	mbs_data->ux[CtrlIndex::RightFootPitch+1]   = r_ankle;
	mbs_data->ux[CtrlIndex::RightFootPitch+nb_mot+1]  = r_ankle_p;

	mbs_data->ux[CtrlIndex::LeftHipPitch+1]   = l_hip;
	mbs_data->ux[CtrlIndex::LeftHipPitch+nb_mot+1]  = l_hip_p;
	mbs_data->ux[CtrlIndex::LeftKneePitch+1]  = l_knee;
	mbs_data->ux[CtrlIndex::LeftKneePitch+nb_mot] = l_knee_p;
	mbs_data->ux[CtrlIndex::LeftFootPitch+1]   = l_ankle;
	mbs_data->ux[CtrlIndex::LeftFootPitch+nb_mot+1]  = l_ankle_p;
}
