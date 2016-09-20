#include "compute_spring_toe.h"
#include "CppInterface.hh"
#include "ModelSimuIndex.hh"
#include "SimuIndex.hh"
#include "user_model.h"

#define DDs 1.0 ///< damping [(Nm s)/rad]
#define KKs 30.0 ///< stifness [Nm/rad]


void compute_spring_toe(MbsData *mbs_data)
{
	int RightToePitch_id, LeftToePitch_id;

	CppInterface *cpp_int;
	ModelSimuIndex *indexes;
	
	cpp_int = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);
	
	indexes = cpp_int->get_simu_ctrl()->get_simu_index();

	RightToePitch_id = indexes->get_mbs_jt(SimuJointIndex::RightToePitch);
	LeftToePitch_id  = indexes->get_mbs_jt(SimuJointIndex::LeftToePitch);

	mbs_data->Qq[RightToePitch_id] = -KKs * mbs_data->q[RightToePitch_id] - DDs * mbs_data->qd[RightToePitch_id];
	mbs_data->Qq[LeftToePitch_id]  = -KKs * mbs_data->q[LeftToePitch_id]  - DDs * mbs_data->qd[LeftToePitch_id];
}
