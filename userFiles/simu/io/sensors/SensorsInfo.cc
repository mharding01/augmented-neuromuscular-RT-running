
#include "SensorsInfo.hh"

extern "C" 
{
	#include "MBSfun.h"
	#include "mbs_project_interface.h"
}

#include "SimuIndex.hh"
#include "user_IO.h"

double speed_fwd_global;

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 */
SensorsInfo::SensorsInfo(MbsData* mbs_data, ModelSimuIndex *simu_index): ComputationSimu(mbs_data, simu_index)
{
	options = mbs_data->user_IO->options;

	coman_model = options->coman_model;

	nb_joints = simu_index->get_nb_joints();

	S_MidWaist_id = simu_index->get_mbs_S(SimuSsensIndex::MidWaist);

	S_RFoots_id = simu_index->get_mbs_S(SimuSsensIndex::RightFoot);
	S_LFoots_id = simu_index->get_mbs_S(SimuSsensIndex::LeftFoot);

	if (coman_model == FLEX_FEET_COMAN)
	{
		S_RFlexProx_id = simu_index->get_mbs_S(SimuSsensIndex::RightFlexProx);
		S_RFlexDist_id = simu_index->get_mbs_S(SimuSsensIndex::RightFlexDist);
		S_LFlexProx_id = simu_index->get_mbs_S(SimuSsensIndex::LeftFlexProx);
		S_LFlexDist_id = simu_index->get_mbs_S(SimuSsensIndex::LeftFlexDist);
	}

	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		S_RToe_id = simu_index->get_mbs_S(SimuSsensIndex::RToe);
		S_LToe_id = simu_index->get_mbs_S(SimuSsensIndex::LToe);
	}
	
	for(int i=0; i<9; i++)
	{
		S_MidWaist_R[i] = 0.0;
		S_RFoots_R[i]   = 0.0;
		S_LFoots_R[i]   = 0.0;
		S_RToe_R[i]   = 0.0;
		S_LToe_R[i]   = 0.0;
	}

	for(int i=0; i<3; i++)
	{
		S_MidWaist_OM[i]  = 0.0;
		S_MidWaist_OMP[i] = 0.0;
		S_MidWaist_P[i]   = 0.0;
		S_MidWaist_V[i]   = 0.0;
		S_RFoots_P[i]     = 0.0;
		S_LFoots_P[i]     = 0.0;
	}

	waist_to_feet = 0.0;

	// Fsens_info structures
	if (coman_model == FLEX_FEET_COMAN)
	{
		F_Sens_RFlexProx = new_Fsens_info();
		F_Sens_RFlexDist = new_Fsens_info();
		F_Sens_LFlexProx = new_Fsens_info();
		F_Sens_LFlexDist = new_Fsens_info();
	}
	else
	{
		F_Sens_RFoot = new_Fsens_info();
		F_Sens_LFoot = new_Fsens_info();
	}
	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		F_Sens_RToe = new_Fsens_info();
		F_Sens_LToe = new_Fsens_info();
	}


	// sensors allocation

	allocate_sensor(&S_MidWaist, nb_joints);
	allocate_sensor(&S_RFoots, nb_joints);
	allocate_sensor(&S_LFoots, nb_joints);

	init_sensor(&S_RFoots, nb_joints);
	init_sensor(&S_LFoots, nb_joints);
	init_sensor(&S_MidWaist, nb_joints);

	if (coman_model == FLEX_FEET_COMAN)
	{
		allocate_sensor(&S_RFlexProx, nb_joints);
		allocate_sensor(&S_RFlexDist, nb_joints);
		allocate_sensor(&S_LFlexProx, nb_joints);
		allocate_sensor(&S_LFlexDist, nb_joints);

		init_sensor(&S_RFlexProx, nb_joints);
		init_sensor(&S_RFlexDist, nb_joints);
		init_sensor(&S_LFlexProx, nb_joints);
		init_sensor(&S_LFlexDist, nb_joints);
	}

	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		allocate_sensor(&S_RToe, nb_joints);
		allocate_sensor(&S_LToe, nb_joints);

		init_sensor(&S_RToe, nb_joints);
		init_sensor(&S_LToe, nb_joints);
	}
}

/*! \brief destructor
 */
SensorsInfo::~SensorsInfo()
{
	// sensors release
	free_sensor(&S_MidWaist);
	free_sensor(&S_RFoots);
	free_sensor(&S_LFoots);

	if (coman_model == FLEX_FEET_COMAN)
	{
		free_sensor(&S_RFlexProx);
		free_sensor(&S_RFlexDist);
		free_sensor(&S_LFlexProx);
		free_sensor(&S_LFlexDist);
	}

	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		free_sensor(&S_RToe);
		free_sensor(&S_LToe);
	}


	// Fsens_info structures
	if (coman_model == FLEX_FEET_COMAN)
	{
		delete F_Sens_RFlexProx;
		delete F_Sens_RFlexDist;
		delete F_Sens_LFlexProx;
		delete F_Sens_LFlexDist;
	}
	else
	{
		delete F_Sens_RFoot;
		delete F_Sens_LFoot;
	}
	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		delete F_Sens_RToe;
		delete F_Sens_LToe;
	}
}

/*! \brief computations
 */
void SensorsInfo::compute()
{
	// --- Sensors init --- //

	init_sensor(&S_RFoots, nb_joints);
	init_sensor(&S_LFoots, nb_joints);
	init_sensor(&S_MidWaist, nb_joints);

	if (coman_model == FLEX_FEET_COMAN)
	{
		init_sensor(&S_RFlexProx, nb_joints);
		init_sensor(&S_RFlexDist, nb_joints);
		init_sensor(&S_LFlexProx, nb_joints);
		init_sensor(&S_LFlexDist, nb_joints);
	}

	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		init_sensor(&S_RToe, nb_joints);
		init_sensor(&S_LToe, nb_joints);
	}


	// --- Getting data from the sensors --- //

	mbs_sensor(&S_MidWaist, mbs_data, S_MidWaist_id);

	mbs_sensor(&S_RFoots, mbs_data, S_RFoots_id);
	mbs_sensor(&S_LFoots, mbs_data, S_LFoots_id);		

	if (coman_model == FLEX_FEET_COMAN)
	{
		mbs_sensor(&S_RFlexProx, mbs_data, S_RFlexProx_id);
		mbs_sensor(&S_RFlexDist, mbs_data, S_RFlexDist_id);
		mbs_sensor(&S_LFlexProx, mbs_data, S_LFlexProx_id);
		mbs_sensor(&S_LFlexDist, mbs_data, S_LFlexDist_id);
	}

	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		mbs_sensor(&S_RToe, mbs_data, S_RToe_id);
		mbs_sensor(&S_LToe, mbs_data, S_LToe_id);
	}

	// rotation matrix and angular velocity for IMU
	S_MidWaist_R[0] = S_MidWaist.R[1][1];
	S_MidWaist_R[1] = S_MidWaist.R[1][2];
	S_MidWaist_R[2] = S_MidWaist.R[1][3];
	S_MidWaist_R[3] = S_MidWaist.R[2][1];
	S_MidWaist_R[4] = S_MidWaist.R[2][2];
	S_MidWaist_R[5] = S_MidWaist.R[2][3];
	S_MidWaist_R[6] = S_MidWaist.R[3][1];
	S_MidWaist_R[7] = S_MidWaist.R[3][2];
	S_MidWaist_R[8] = S_MidWaist.R[3][3];

	for(int i=0; i<3; i++)
	{
		S_MidWaist_OM[i]  = S_MidWaist.OM[1+i];
		S_MidWaist_OMP[i] = S_MidWaist.OMP[1+i];
	}


	// MidWaist position
	for(int i=0; i<3; i++)
	{
		S_MidWaist_P[i] = S_MidWaist.P[1+i];
		S_MidWaist_V[i] = S_MidWaist.V[1+i];
	}

	speed_fwd_global = S_MidWaist_V[0];


	// right foot
	for(int i=0; i<3; i++)
	{
		S_RFoots_P[i] = S_RFoots.P[1+i];
	}

	S_RFoots_R[0] = S_RFoots.R[1][1];
	S_RFoots_R[1] = S_RFoots.R[1][2];
	S_RFoots_R[2] = S_RFoots.R[1][3];
	S_RFoots_R[3] = S_RFoots.R[2][1];
	S_RFoots_R[4] = S_RFoots.R[2][2];
	S_RFoots_R[5] = S_RFoots.R[2][3];
	S_RFoots_R[6] = S_RFoots.R[3][1];
	S_RFoots_R[7] = S_RFoots.R[3][2];
	S_RFoots_R[8] = S_RFoots.R[3][3];

	S_RToe_R[0] = S_RToe.R[1][1];
	S_RToe_R[1] = S_RToe.R[1][2];
	S_RToe_R[2] = S_RToe.R[1][3];
	S_RToe_R[3] = S_RToe.R[2][1];
	S_RToe_R[4] = S_RToe.R[2][2];
	S_RToe_R[5] = S_RToe.R[2][3];
	S_RToe_R[6] = S_RToe.R[3][1];
	S_RToe_R[7] = S_RToe.R[3][2];
	S_RToe_R[8] = S_RToe.R[3][3];

	// left foot
	for(int i=0; i<3; i++)
	{
		S_LFoots_P[i] = S_LFoots.P[1+i];
	}

	S_LFoots_R[0] = S_LFoots.R[1][1];
	S_LFoots_R[1] = S_LFoots.R[1][2];
	S_LFoots_R[2] = S_LFoots.R[1][3];
	S_LFoots_R[3] = S_LFoots.R[2][1];
	S_LFoots_R[4] = S_LFoots.R[2][2];
	S_LFoots_R[5] = S_LFoots.R[2][3];
	S_LFoots_R[6] = S_LFoots.R[3][1];
	S_LFoots_R[7] = S_LFoots.R[3][2];
	S_LFoots_R[8] = S_LFoots.R[3][3];

	S_LToe_R[0] = S_LToe.R[1][1];
	S_LToe_R[1] = S_LToe.R[1][2];
	S_LToe_R[2] = S_LToe.R[1][3];
	S_LToe_R[3] = S_LToe.R[2][1];
	S_LToe_R[4] = S_LToe.R[2][2];
	S_LToe_R[5] = S_LToe.R[2][3];
	S_LToe_R[6] = S_LToe.R[3][1];
	S_LToe_R[7] = S_LToe.R[3][2];
	S_LToe_R[8] = S_LToe.R[3][3];


	// waist to feet distance
	waist_to_feet   = S_MidWaist_P[2] - ((S_RFoots_P[2] < S_LFoots_P[2]) ? S_RFoots_P[2] : S_LFoots_P[2]);

	// Fsens_info structures
	if (coman_model == FLEX_FEET_COMAN)
	{
		fill_Fsens_info(S_RFlexProx, F_Sens_RFlexProx);
		fill_Fsens_info(S_RFlexDist, F_Sens_RFlexDist);
		fill_Fsens_info(S_LFlexProx, F_Sens_LFlexProx);
		fill_Fsens_info(S_LFlexDist, F_Sens_LFlexDist);
	}
	else
	{
		fill_Fsens_info(S_RFoots, F_Sens_RFoot);
		fill_Fsens_info(S_LFoots, F_Sens_LFoot);
	}
	if ((coman_model == SPRING_TOE_SHORT_FEET_COMAN) || (coman_model == SPRING_TOE_FEET_COMAN))
	{
		fill_Fsens_info(S_RToe, F_Sens_RToe);
		fill_Fsens_info(S_LToe, F_Sens_LToe);
	}
}

/*! \brief print the position and the velocity of a sensor
 * 
 * \param[in] sens_id sensor ID
 * \param[in] rel_waist 1 to get kinematics relative to the waist, 0 otherwise
 */
void SensorsInfo::sensor_kinematics(int sens_id, int rel_waist)
{
	MbsSensor this_sens;

	allocate_sensor(&this_sens, nb_joints);
	init_sensor(&this_sens, nb_joints);

	mbs_sensor(&this_sens, mbs_data, sens_id);

	if (rel_waist)
	{
		std::cout << "position: " << this_sens.P[1]-S_MidWaist_P[0] << " ; " << this_sens.P[2]-S_MidWaist_P[1] << " ; " << this_sens.P[3]-S_MidWaist_P[2] << std::endl;
		std::cout << "velocity: " << this_sens.V[1]-S_MidWaist_V[0] << " ; " << this_sens.V[2]-S_MidWaist_V[1] << " ; " << this_sens.V[3]-S_MidWaist_V[2] << std::endl;
	}
	else
	{
		std::cout << "position: " << this_sens.P[1] << " ; " << this_sens.P[2] << " ; " << this_sens.P[3] << std::endl;
		std::cout << "velocity: " << this_sens.V[1] << " ; " << this_sens.V[2] << " ; " << this_sens.V[3] << std::endl;
	}

	free_sensor(&this_sens);
}

/*! \brief print the position and the velocity relative to two sensors
 * 
 * \param[in] sens_1 sensor ID of the first sensor
 * \param[in] sens_2 sensor ID of the second sensor
 */
void SensorsInfo::rel_sensor_kinematics(int sens_1, int sens_2)
{
	MbsSensor first_sens, second_sens;

	allocate_sensor(&first_sens , nb_joints);
	allocate_sensor(&second_sens, nb_joints);

	init_sensor(&first_sens , nb_joints);
	init_sensor(&second_sens, nb_joints);

	mbs_sensor(&first_sens, mbs_data , sens_1);
	mbs_sensor(&second_sens, mbs_data, sens_2);

	std::cout << "position: " << first_sens.P[1]-second_sens.P[1] << " ; " << first_sens.P[2]-second_sens.P[2] << " ; " << first_sens.P[3]-second_sens.P[3] << std::endl;
	std::cout << "velocity: " << first_sens.V[1]-second_sens.V[1] << " ; " << first_sens.V[2]-second_sens.V[2] << " ; " << first_sens.V[3]-second_sens.V[3] << std::endl;

	free_sensor(&first_sens);
	free_sensor(&second_sens);
}

/*! \brief initialize the COM and the bodies to compute the global COM
 */
void SensorsInfo::com_body_init()
{
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::WaistCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RHipmotCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RThighUpLegCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RThighLowLegCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RLowLegCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RFootmotCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RFootCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LHipmotCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LThighUpLegCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LThighLowLegCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LLowLegCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LFootmotCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LFootCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::DWLCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::DWSCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::DWYCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RShpCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RShrCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RShyCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::RElbCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LShpCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LShrCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LShyCg));
	com_ids.push_back(simu_index->get_mbs_S(SimuSsensIndex::LElbCg));

	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::Waist));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RHipmot));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RThighUpLeg));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RThighLowLeg));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RLowLeg));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RFootmot));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RFoot));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LHipmot));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LThighUpLeg));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LThighLowLeg));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LLowLeg));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LFootmot));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LFoot));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::DWL));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::DWS));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::DWYTorso));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RShp));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RShr));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RShy));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::RElb));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LShp));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LShr));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LShy));
	body_ids.push_back(simu_index->get_mbs_bd(SimuBodyIndex::LElb));
}

/*! \brief print the position and velocitiy of the center of mass
 *
 * \param[in] rel_waist 1 to get kinematics relative to the waist, 0 otherwise
 */
void SensorsInfo::com_kinematics(int rel_waist)
{
	// variables declaration
	MbsSensor cur_sens;
	double r_com[3], rp_com[3];
	double cur_mass, tot_mass;

	// variables initialization
	for(int i=0; i<3; i++)
	{
		r_com[i]  = 0.0;
		rp_com[i] = 0.0;
	}

	tot_mass = 0.0;

	allocate_sensor(&cur_sens, nb_joints);
	init_sensor(&cur_sens, nb_joints);

	// loop on all bodies
	for(unsigned int i=0; i<com_ids.size(); i++)
	{
		mbs_sensor(&cur_sens, mbs_data, com_ids[i]);

		cur_mass  = mbs_data->m[body_ids[i]];
		tot_mass += cur_mass;

		for(int j=0; j<3; j++)
		{
			r_com[j]  += cur_mass * cur_sens.P[j+1];
			rp_com[j] += cur_mass * cur_sens.V[j+1];
		}
	}

	for(int i=0; i<3; i++)
	{
		r_com[i]  /= tot_mass;
		rp_com[i] /= tot_mass;
	}

	if (rel_waist)
	{
		std::cout << "COM position: " << r_com[0]  - S_MidWaist_P[0] << " ; " << r_com[1]  - S_MidWaist_P[1] << " ; " << r_com[2]  - S_MidWaist_P[2] << std::endl;
		std::cout << "COM velocity: " << rp_com[0] - S_MidWaist_V[0] << " ; " << rp_com[1] - S_MidWaist_V[1] << " ; " << rp_com[2] - S_MidWaist_V[2] << std::endl;
	}
	else
	{
		std::cout << "COM position: " << r_com[0]  << " ; " << r_com[1]  << " ; " << r_com[2]  << std::endl;
		std::cout << "COM velocity: " << rp_com[0] << " ; " << rp_com[1] << " ; " << rp_com[2] << std::endl;
	}

	free_sensor(&cur_sens);
}

/*! \brief create a Fsens_info struct and set all its fiels to 0
 * 
 * \return Fsens_info struct allocated and reset to 0
 */
Fsens_info* SensorsInfo::new_Fsens_info()
{
	Fsens_info *f_sens_info = new Fsens_info();

	for(int i=0; i<3; i++)
	{
		f_sens_info->P[i]  = 0.0;
		f_sens_info->V[i]  = 0.0;
		f_sens_info->OM[i] = 0.0;
	}

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			f_sens_info->R[i][j] = 0.0;
		}
	}

	return f_sens_info;
}

/*! \brief fill a Fsens_info struct
 * 
 * \param[in] cur_Fsens current MbsSensor info structure filled by Robotran
 * \param[out] cur_Fsens_info Fsens_info struct to fill
 */
void SensorsInfo::fill_Fsens_info(MbsSensor cur_Fsens, Fsens_info *cur_Fsens_info)
{
	for(int i=0; i<3; i++)
	{
		cur_Fsens_info->P[i]  = cur_Fsens.P[1+i];
		cur_Fsens_info->V[i]  = cur_Fsens.V[1+i];
		cur_Fsens_info->OM[i] = cur_Fsens.OM[1+i];
	}

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			cur_Fsens_info->R[i][j] = cur_Fsens.R[1+i][1+j];
		}
	}
}
