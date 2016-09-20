#include "Body.hh"
#include "StimGeyerCtrl.hh"
#include "StimWangCtrl.hh"
#include "StimWalkCtrl.hh"
#include "StimUpperWalk.hh"
#include "StimMin.hh"
#include "StimQqRefCtrl.hh"
#include "StimTrapezoid.hh"
#include "CtrlIndex.hh"
#include "InverseKinLeg.hh"
#include "Leg.hh"
#include "Torso.hh"
#include "Arm.hh"
#include <stdlib.h>

#include "cmake_config.h"
#include <fstream>

#define RAD_TO_DEG (180.0/M_PI)

inline int getmod(int index, int index_max) { return ((index % index_max) + index_max) % index_max; }


/*! \brief constructor
 * 
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] outputs controller outputs
 * \param[in] ws walk states
 * \param[in] fwd_kin forward kinematics
 */
Body::Body(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs,
	WalkStates *ws, ForwardKinematics *fwd_kin): Computation(inputs, options, ctrl_index)
{
	int flag_ctrl;

	this->outputs = outputs;
	this->ws = ws;
	this->fwd_kin = fwd_kin;

	t = inputs->get_t();

	m_st = static_cast<MainState*>(ws->get_state(MAIN_STATE));

	flag_ctrl = options->get_flag_ctrl();

	flag_apply_Qq_wang = options->is_apply_Qq_wang();

	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		switch (i)
		{
			case RIGHT_LEG_BODY:
			case LEFT_LEG_BODY:
				parts[i] = new Leg(inputs, ctrl_index, i);
				break;

			case TORSO_BODY:
				parts[i] = new Torso(inputs, ctrl_index);
				break;

			case RIGHT_ARM_BODY:
			case LEFT_ARM_BODY:
				parts[i] = new Arm(inputs, ctrl_index, i);
				break;
		
			default:
				std::cout << "Error: unknown body part : " << i << " !" << std::endl;
				exit(EXIT_FAILURE);
		}
	}

	stim_init = new StimQqRefCtrl(inputs, ws, fwd_kin, parts, options);

	switch (flag_ctrl)
	{
		case CTRL_CPG:
			stim_ctrl = new StimWalkCtrl(inputs, ws, fwd_kin, parts, options);
			break;

		case CTRL_REFLEX:
			stim_ctrl = new StimGeyerCtrl(inputs, ws, fwd_kin, parts, options);
			break;

		case CTRL_REFLEX_WANG:
			stim_ctrl = new StimWangCtrl(inputs, ws, fwd_kin, parts, options);
			break;

		case CTRL_STIM_TEST:
			stim_ctrl = new StimTrapezoid(inputs, ws, fwd_kin, parts, options);
			break;

		default:
			std::cout << "Error: bad flag_ctrl: " << flag_ctrl << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}

	//stim_upper = new StimMin(inputs, ws, fwd_kin, parts, options);
	stim_upper = new StimUpperWalk(inputs, ws, fwd_kin, parts, options, ctrl_index, stim_ctrl);
	
	// i/o vector IDs
	RightHipPitch_id  = ctrl_index->get_inv_index(CtrlIndex::RightHipPitch);
	RightHipRoll_id   = ctrl_index->get_inv_index(CtrlIndex::RightHipRoll);
	RightHipYaw_id    = ctrl_index->get_inv_index(CtrlIndex::RightHipYaw);
	RightKneePitch_id = ctrl_index->get_inv_index(CtrlIndex::RightKneePitch);
	RightFootRoll_id  = ctrl_index->get_inv_index(CtrlIndex::RightFootRoll);
	RightFootPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightFootPitch);

	LeftHipPitch_id  = ctrl_index->get_inv_index(CtrlIndex::LeftHipPitch);
	LeftHipRoll_id   = ctrl_index->get_inv_index(CtrlIndex::LeftHipRoll);
	LeftHipYaw_id    = ctrl_index->get_inv_index(CtrlIndex::LeftHipYaw);
	LeftKneePitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftKneePitch);
	LeftFootRoll_id  = ctrl_index->get_inv_index(CtrlIndex::LeftFootRoll);
	LeftFootPitch_id = ctrl_index->get_inv_index(CtrlIndex::LeftFootPitch);

	RightShPitch_id  = ctrl_index->get_inv_index(CtrlIndex::RightShPitch);
	RightShRoll_id	 = ctrl_index->get_inv_index(CtrlIndex::RightShRoll);
	RightShYaw_id	 = ctrl_index->get_inv_index(CtrlIndex::RightShYaw);
	RightElbPitch_id = ctrl_index->get_inv_index(CtrlIndex::RightElbPitch);

	LeftShPitch_id	 = ctrl_index->get_inv_index(CtrlIndex::LeftShPitch);
	LeftShRoll_id    = ctrl_index->get_inv_index(CtrlIndex::LeftShRoll);
	LeftShYaw_id     = ctrl_index->get_inv_index(CtrlIndex::LeftShYaw);
	LeftElbPitch_id  = ctrl_index->get_inv_index(CtrlIndex::LeftElbPitch);
	
	TorsoRoll_id     = ctrl_index->get_inv_index(CtrlIndex::TorsoRoll);
	TorsoPitch_id    = ctrl_index->get_inv_index(CtrlIndex::TorsoPitch);
	TorsoYaw_id      = ctrl_index->get_inv_index(CtrlIndex::TorsoYaw);

	// metabolic energy
	met_energy_total = 0.0;

	if (flag_apply_Qq_wang)
	{
		//load torque data from Wang
		std::ifstream ihip(PROJECT_SOURCE_DIR"/../userFiles/dataWang/hipQq.txt", std::ios::in);
		std::ifstream iknee(PROJECT_SOURCE_DIR"/../userFiles/dataWang/kneeQq.txt", std::ios::in);
		std::ifstream iankle(PROJECT_SOURCE_DIR"/../userFiles/dataWang/ankleQq.txt", std::ios::in);
		if (!ihip.is_open() || !iknee.is_open() || !iankle.is_open())
		{
		  	std::cout << "Error: cant open wang hip files" << std::endl;
		  	exit(1);
		}
		double num = 0.0;
		while (ihip >> num)
			RHipQq.push_back(num);
		   num = 0.0;
		while (iknee >> num)
			RKneeQq.push_back(num);
		num = 0.0;
		while (iankle >> num)
			RAnkleQq.push_back(num);

		//param
		arh = 1.0;
		ark = 1.0;
		ara = 1.0;
		crh = 1.0;
		crk = 1.0;
		cra = 1.0;
		brh = 0.0;
		brk = 0.0;
		bra = 0.0;

		alh = 1.0;
		alk = 1.0;
		ala = 1.0;
		clh = 1.0;
		clk = 1.0;
		cla = 1.0;
		blh = 0.0;
		blk = 0.0;
		bla = 0.0;

		inputs->get_opti_inputs()->set_body(this);
	}
}

/*! \brief destructor
 */
Body::~Body()
{
	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		delete parts[i];
	}

	delete stim_init;
	delete stim_ctrl;
	delete stim_upper;
}

/*! \brief main computation
 */
void Body::compute()
{
	StimulationCtrl *cur_stim_ctrl;

	t = inputs->get_t();

	met_energy_total = 0.0;

	switch (m_st->get_coman_state())
	{
		case INIT_UPRIGHT_STATE :
			cur_stim_ctrl = stim_init;
			break;

		case TEST_STATE :
		case WALK_COMAN_STATE :
			cur_stim_ctrl = stim_ctrl;
			break;

		default :
			std::cout << "Error: unknown COMAN state: " << m_st->get_coman_state() << " !" << std::endl;
			exit(EXIT_FAILURE);
	}

	// update inputs
	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		parts[i]->update_inputs();
	}

	// compute stimulation
	cur_stim_ctrl->compute();
	stim_upper->compute();
	
	// apply stimulations
	for(int i=0; i<NB_BODY_PARTS; i++)
	{
		switch (i)
		{
			case RIGHT_LEG_BODY:
			case LEFT_LEG_BODY:
				parts[i]->update_body_part(cur_stim_ctrl);
				break;

			case TORSO_BODY:
			case RIGHT_ARM_BODY:
			case LEFT_ARM_BODY:
				parts[i]->update_body_part(stim_upper);
				break;
		
			default:
				std::cout << "Error: unknown body part : " << i << " !" << std::endl;
				exit(EXIT_FAILURE);
		}
		met_energy_total += parts[i]->get_met_energy();
	}

	// send torque references
	send_references();
}

/*! \brief send references to main controller
 */
void Body::send_references()
{	
	//-----legs-----
	if (flag_apply_Qq_wang)
	{
		double index = inputs->get_t()/125.0e-6;
		double Qq_ratio = 28.3*42.7/93.0;
		double length_cycle = RHipQq.size()/2.0;

		// pitch
		outputs->set_Qq_ref(RightHipPitch_id,  arh*RHipQq[  getmod((int)(crh*index)+(int)(brh*length_cycle),length_cycle)]*Qq_ratio);
		outputs->set_Qq_ref(RightKneePitch_id, ark*RKneeQq[ getmod((int)(crk*index)+(int)(brk*length_cycle),length_cycle)]*Qq_ratio);
		outputs->set_Qq_ref(RightFootPitch_id, ara*RAnkleQq[getmod((int)(cra*index)+(int)(bra*length_cycle),length_cycle)]*Qq_ratio);

		outputs->set_Qq_ref(LeftHipPitch_id,  alh*RHipQq[  getmod((int)(clh*index)+(int)(blh*length_cycle),length_cycle)]*Qq_ratio);
		outputs->set_Qq_ref(LeftKneePitch_id, alk*RKneeQq[ getmod((int)(clk*index)+(int)(blk*length_cycle),length_cycle)]*Qq_ratio);
		outputs->set_Qq_ref(LeftFootPitch_id, ala*RAnkleQq[getmod((int)(cla*index)+(int)(bla*length_cycle),length_cycle)]*Qq_ratio);
	}
	else
	{
		// pitch
		outputs->set_Qq_ref(RightHipPitch_id,  parts[RIGHT_LEG_BODY]->get_Qq(PITCH_HIP_ART));
		outputs->set_Qq_ref(RightKneePitch_id, parts[RIGHT_LEG_BODY]->get_Qq(PITCH_KNEE_ART));
		outputs->set_Qq_ref(RightFootPitch_id, parts[RIGHT_LEG_BODY]->get_Qq(PITCH_FOOT_ART));

		outputs->set_Qq_ref(LeftHipPitch_id,  parts[LEFT_LEG_BODY]->get_Qq(PITCH_HIP_ART));
		outputs->set_Qq_ref(LeftKneePitch_id, parts[LEFT_LEG_BODY]->get_Qq(PITCH_KNEE_ART));
		outputs->set_Qq_ref(LeftFootPitch_id, parts[LEFT_LEG_BODY]->get_Qq(PITCH_FOOT_ART));
	}

	// roll
	outputs->set_Qq_ref(RightHipRoll_id,  parts[RIGHT_LEG_BODY]->get_Qq(ROLL_HIP_ART));
	outputs->set_Qq_ref(RightFootRoll_id, parts[RIGHT_LEG_BODY]->get_Qq(ROLL_FOOT_ART));

	outputs->set_Qq_ref(LeftHipRoll_id,  parts[LEFT_LEG_BODY]->get_Qq(ROLL_HIP_ART));
	outputs->set_Qq_ref(LeftFootRoll_id, parts[LEFT_LEG_BODY]->get_Qq(ROLL_FOOT_ART));

	// yaw
	outputs->set_Qq_ref(RightHipYaw_id, parts[RIGHT_LEG_BODY]->get_Qq(YAW_HIP_ART));
	outputs->set_Qq_ref(LeftHipYaw_id,  parts[LEFT_LEG_BODY]->get_Qq(YAW_HIP_ART));


	//-----torso----
	outputs->set_Qq_ref(TorsoPitch_id, parts[TORSO_BODY]->get_Qq(PITCH_TORSO_ART)); 	
	outputs->set_Qq_ref(TorsoRoll_id,  parts[TORSO_BODY]->get_Qq(ROLL_TORSO_ART));		
	outputs->set_Qq_ref(TorsoYaw_id,   parts[TORSO_BODY]->get_Qq(YAW_TORSO_ART));

	//-----arms----
	outputs->set_Qq_ref(RightShPitch_id,  parts[RIGHT_ARM_BODY]->get_Qq(PITCH_SHOULDER_ART));
	outputs->set_Qq_ref(RightShRoll_id,   parts[RIGHT_ARM_BODY]->get_Qq(ROLL_SHOULDER_ART));
	outputs->set_Qq_ref(RightShYaw_id,    parts[RIGHT_ARM_BODY]->get_Qq(YAW_SHOULDER_ART));
	outputs->set_Qq_ref(RightElbPitch_id, parts[RIGHT_ARM_BODY]->get_Qq(PITCH_ELBOW_ART));

	outputs->set_Qq_ref(LeftShPitch_id,  parts[LEFT_ARM_BODY]->get_Qq(PITCH_SHOULDER_ART));
	outputs->set_Qq_ref(LeftShRoll_id,   parts[LEFT_ARM_BODY]->get_Qq(ROLL_SHOULDER_ART));
	outputs->set_Qq_ref(LeftShYaw_id,    parts[LEFT_ARM_BODY]->get_Qq(YAW_SHOULDER_ART));
	outputs->set_Qq_ref(LeftElbPitch_id, parts[LEFT_ARM_BODY]->get_Qq(PITCH_ELBOW_ART));
}
