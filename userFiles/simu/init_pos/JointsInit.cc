
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
	T3   = 0.51223863;		// Taken from workR/Optis_3.0/1.5vt8.3/OptiResults
	T3_p =  0.00185412;
	R2   = 0.06613346;
	R2_p = 3.91008657;

	r_sh   = 0.0;
	r_sh_p = 4.87004676;

	r_hip     = -0.77071918;
	r_hip_p   = -3.83071851;
	r_knee    = 0.24857749;
	r_knee_p  = -3.64650428;
	r_ankle   = 0.07297330;
	r_ankle_p = -3.85941459;

	l_hip     = 0.06109374;
	l_hip_p   = -1.23523796;
	l_knee    = 1.22608835;
	l_knee_p  = 3.78344275;
	l_ankle   = 0.30323805;
	l_ankle_p = 4.18894087;

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
