#include "ImpedanceCtrlInit.hh"
#include "ctrl_functions.hh"

/*! \brief constructor
 *
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] outputs controller outputs
 * \param[in] fwd_kin forward kinematics
 */
ImpedanceCtrlInit::ImpedanceCtrlInit(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin):
	ImpedanceCtrlStanding(inputs, options, ctrl_index, outputs, fwd_kin)
{
		// COM position
	r_COM_ref[0] = 0.0;
	r_COM_ref[1] = 0.0;
	r_COM_ref[2] = 0.56;

	// gains
	kp_x = 250.0;
	kp_y = 500.0;
	kp_z = 2200.0;

	kd_x = 100.0;
	kd_y = 100.0;
	kd_z = 50.0;

	ki_x = 0.0;
	ki_y = 0.0;
	ki_z = 0.0;

	t = inputs->get_t();

	r_first_swing = options->is_r_first_swing();
	flag_3D = options->is_flag_3D();

	x_com_foot_ref = 0.0;
	y_com_foot_ref = 0.0;

	x_ref_init = 0.0;
	y_ref_init = 0.0;

	x_ref_end = 0.01750616;
	y_ref_end = 0.01;

	flag_first = 0;

	init_t = 0.0;

	// for optimization
	inputs->get_opti_inputs()->set_impedance_init(this);
}

/*! \brief destructor
 */
ImpedanceCtrlInit::~ImpedanceCtrlInit()
{

}

/*! \brief main computation
 */
void ImpedanceCtrlInit::compute()
{
	// current time
	t = inputs->get_t();

	// first call
	if (!flag_first)
	{
		flag_first = 1;

		init_t = inputs->get_t();

		compute_GAF();

		if (r_first_swing)
		{
			x_ref_init = -(io->r_Lfoot[0] - r_base[0]);
			y_ref_init = -(io->r_Lfoot[1] - r_base[1]);
		}
		else
		{
			x_ref_init = -(io->r_Rfoot[0] - r_base[0]);
			y_ref_init = -(io->r_Rfoot[1] - r_base[1]);
		}
	}

	// x-y COM reference position
	x_com_foot_ref = linear_interpol(x_ref_init, x_ref_end, init_t, 1.0, t);

	if (r_first_swing)
	{
		y_com_foot_ref = linear_interpol(y_ref_init, -y_ref_end, init_t, 1.0, t);

		r_COM_ref[0] = (io->r_Lfoot[0] - r_base[0]) + x_com_foot_ref;
		r_COM_ref[1] = (io->r_Lfoot[1] - r_base[1]) + y_com_foot_ref;
	}
	else
	{
		y_com_foot_ref = linear_interpol(y_ref_init, y_ref_end, init_t, 1.0, t);

		r_COM_ref[0] = (io->r_Rfoot[0] - r_base[0]) + x_com_foot_ref;
		r_COM_ref[1] = (io->r_Rfoot[1] - r_base[1]) + y_com_foot_ref;
	}

	// 2D walking
	if (!flag_3D)
	{
		r_COM_ref[1] = 0.0;
	}

	// main computation
	ImpedanceCtrl::compute();
}
