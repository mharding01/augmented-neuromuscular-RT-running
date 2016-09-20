#include "NicoCtrl.hh"
#include "CtrlIndex.hh"
#include "CoManIndex.hh"
#include "random_ctrl.hh"

/*! \brief constructor
 */
NicoCtrl::NicoCtrl(): Ctrl(NICO_CTRL)
{
	struct timeval seed_time;
	
	// seed for random
	gettimeofday(&seed_time, NULL);
	srand(seed_time.tv_usec * seed_time.tv_sec);

	options     = new CtrlOptions();
	motor_index = new MotorCtrlIndex(options);

	inputs  = new CtrlInputs(user_ctrl, motor_index, options);
	outputs = new CtrlOutputs(motor_index);
	manager = new ComputeManager(inputs, outputs, options, motor_index);

	//inputs->get_opti_inputs()->set_opti();
}

/*! \brief desctructor
 */
NicoCtrl::~NicoCtrl()
{
	delete options;
	delete motor_index;
	delete inputs;
	delete outputs;
	delete manager;
}

/*! \brief controller initialization
 */
void NicoCtrl::init_ctrl()
{

}

/*! \brief controller main loop
 */
void NicoCtrl::loop_ctrl()
{
	manager->run_computations();
}

/*! \brief controller finalization
 */
void NicoCtrl::finish_ctrl()
{

}

/*! \brief controller inputs filling
 * 
 * \param[in] inputsCtrl controller inputs structure
 *
 * For more information about the controller inputs, src/project/interface_controller/controller_io.hh.
 * Fill your controller inputs, reading the ones from 'inputsCtrl'.
 */
void NicoCtrl::set_inputs(Inputs_ctrl *inputsCtrl)
{
	int ctrl_index, coman_index;

	// time
	inputs->set_t(inputsCtrl->t);

	// positions - velocities - torques

	for (int i=0; i<motor_index->get_nb_mot(); i++)
	{
		ctrl_index  = motor_index->get_index(i);
		coman_index = get_coman_index(ctrl_index);

		inputs->set_q( i, inputsCtrl->q[coman_index]);
		inputs->set_qd(i, inputsCtrl->qd[coman_index]);
		inputs->set_Qq(i, inputsCtrl->Qq[coman_index]);

		inputs->set_q_mot( i, inputsCtrl->q[coman_index]);
		inputs->set_qd_mot(i, inputsCtrl->qd[coman_index]);
	}

	// feet (forces - torques) and IMU
	for (int i=0; i<3; i++)
	{
		inputs->set_F_Rfoot(i, inputsCtrl->F_Rfoot[i]);
		inputs->set_F_Lfoot(i, inputsCtrl->F_Lfoot[i]);
		inputs->set_F_Rtoe_IF(i, inputsCtrl->F_RToe_IF[i]);
		inputs->set_F_Ltoe_IF(i, inputsCtrl->F_LToe_IF[i]);
		inputs->set_T_Rfoot(i, inputsCtrl->T_Rfoot[i]);
		inputs->set_T_Lfoot(i, inputsCtrl->T_Lfoot[i]);

		inputs->set_IMU_Angular_Rate(i, inputsCtrl->IMU_Angular_Rate[i]);
		inputs->set_IMU_Acceleration(i, inputsCtrl->IMU_Acceleration[i]);
	}

	// IMU orientation
	for(int i=0; i<9; i++)
	{
		inputs->set_IMU_Orientation(i, inputsCtrl->IMU_Orientation[i]);
	}
}

/*! \brief controller outputs filling
 * 
 * \param[out] outputsCtrl controller outputs structure
 *
 * For more information about the controller outputs, src/project/interface_controller/controller_io.hh.
 * Fill 'outputsCtrl' with your controller outputs.
 */
void NicoCtrl::set_outputs(Outputs_ctrl *outputsCtrl)
{
	int ctrl_index, coman_index;

	// tracking (position - velocity - torque)
	
	for (int i=0; i<motor_index->get_nb_mot(); i++)
	{	
		ctrl_index  = motor_index->get_index(i);
		coman_index = get_coman_index(ctrl_index);
		
		outputsCtrl->q_ref[coman_index]  = outputs->get_q_ref(i);
		outputsCtrl->Qq_ref[coman_index] = outputs->get_Qq_ref(i);
	}
}

/*! \brief translate interface index to implemented motors controller one
 * 
 * \param[in] coman_index index of the COMAN's controller interface
 * \return implemented motors controller index (-1 if invalid)
 */
int NicoCtrl::get_ctrl_index(int coman_index)
{
	switch (coman_index)
	{
		// right leg
		case CoManIndex::RightHipPitch  : return CtrlIndex::RightHipPitch;
		case CoManIndex::RightHipRoll   : return CtrlIndex::RightHipRoll;
		case CoManIndex::RightHipYaw    : return CtrlIndex::RightHipYaw;
		case CoManIndex::RightKneePitch : return CtrlIndex::RightKneePitch;
		case CoManIndex::RightFootRoll  : return CtrlIndex::RightFootRoll;
		case CoManIndex::RightFootPitch : return CtrlIndex::RightFootPitch;

		// left leg
		case CoManIndex::LeftHipPitch  : return CtrlIndex::LeftHipPitch;
		case CoManIndex::LeftHipRoll   : return CtrlIndex::LeftHipRoll;
		case CoManIndex::LeftHipYaw    : return CtrlIndex::LeftHipYaw;
		case CoManIndex::LeftKneePitch : return CtrlIndex::LeftKneePitch;
		case CoManIndex::LeftFootRoll  : return CtrlIndex::LeftFootRoll;
		case CoManIndex::LeftFootPitch : return CtrlIndex::LeftFootPitch;

		// torso
		case CoManIndex::TorsoRoll  : return CtrlIndex::TorsoRoll;
		case CoManIndex::TorsoPitch : return CtrlIndex::TorsoPitch;
		case CoManIndex::TorsoYaw   : return CtrlIndex::TorsoYaw;

		// right arm
		case CoManIndex::RightShPitch  : return CtrlIndex::RightShPitch;
		case CoManIndex::RightShRoll   : return CtrlIndex::RightShRoll;
		case CoManIndex::RightShYaw    : return CtrlIndex::RightShYaw;
		case CoManIndex::RightElbPitch : return CtrlIndex::RightElbPitch;

		// left arm
		case CoManIndex::LeftShPitch  : return CtrlIndex::LeftShPitch;
		case CoManIndex::LeftShRoll   : return CtrlIndex::LeftShRoll;
		case CoManIndex::LeftShYaw    : return CtrlIndex::LeftShYaw;
		case CoManIndex::LeftElbPitch : return CtrlIndex::LeftElbPitch;
	
		default:
			std::cout << "Warning: requested unvalid interface index: " << coman_index << std::endl;
			return -1;
	}
}

/*! \brief translate implemented motors controller index to interface one
 * 
 * \param[in] ctrl_index index from implemented motors controller interface
 * \return interface controller index (-1 if invalid)
 */
int NicoCtrl::get_coman_index(int ctrl_index)
{
	switch (ctrl_index)
	{
		// right leg
		case CtrlIndex::RightHipPitch  : return CoManIndex::RightHipPitch;
		case CtrlIndex::RightHipRoll   : return CoManIndex::RightHipRoll;
		case CtrlIndex::RightHipYaw    : return CoManIndex::RightHipYaw;
		case CtrlIndex::RightKneePitch : return CoManIndex::RightKneePitch;
		case CtrlIndex::RightFootRoll  : return CoManIndex::RightFootRoll;
		case CtrlIndex::RightFootPitch : return CoManIndex::RightFootPitch;

		// left leg
		case CtrlIndex::LeftHipPitch  : return CoManIndex::LeftHipPitch;
		case CtrlIndex::LeftHipRoll   : return CoManIndex::LeftHipRoll;
		case CtrlIndex::LeftHipYaw    : return CoManIndex::LeftHipYaw;
		case CtrlIndex::LeftKneePitch : return CoManIndex::LeftKneePitch;
		case CtrlIndex::LeftFootRoll  : return CoManIndex::LeftFootRoll;
		case CtrlIndex::LeftFootPitch : return CoManIndex::LeftFootPitch;

		// torso
		case CtrlIndex::TorsoRoll  : return CoManIndex::TorsoRoll;
		case CtrlIndex::TorsoPitch : return CoManIndex::TorsoPitch;
		case CtrlIndex::TorsoYaw   : return CoManIndex::TorsoYaw;

		// right arm
		case CtrlIndex::RightShPitch  : return CoManIndex::RightShPitch;
		case CtrlIndex::RightShRoll   : return CoManIndex::RightShRoll;
		case CtrlIndex::RightShYaw    : return CoManIndex::RightShYaw;
		case CtrlIndex::RightElbPitch : return CoManIndex::RightElbPitch;

		// left arm
		case CtrlIndex::LeftShPitch  : return CoManIndex::LeftShPitch;
		case CtrlIndex::LeftShRoll   : return CoManIndex::LeftShRoll;
		case CtrlIndex::LeftShYaw    : return CoManIndex::LeftShYaw;
		case CtrlIndex::LeftElbPitch : return CoManIndex::LeftElbPitch;

		default:
			std::cout << "Warning: requested unvalid implemented motors controller index: " << ctrl_index << std::endl;
			return -1;
	}
}
