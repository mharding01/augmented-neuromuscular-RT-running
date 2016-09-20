#include "ImpedanceCtrl.hh"
#include "coman_properties.hh"
#include "CtrlIndex.hh"

/// limit range
inline double limit(double x, double min, double max) { return (x < min) ? min : (x > max) ? max : x; }

/*! \brief constructor
 *
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] outputs controller outputs
 * \param[in] fwd_kin forward kinematics
 * \param[in] nb_cont number of constraints [⁻]
 * \param[in] nb_mot number of motors [-]
 */
ImpedanceCtrl::ImpedanceCtrl(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin, int nb_cont, int nb_mot):
	Computation(inputs, options, ctrl_index), fs(3, nb_cont),
	jacob_x(nb_mot, nb_cont), jacob_y(nb_mot, nb_cont), jacob_z(nb_mot, nb_cont),
	Ax(2, nb_cont),        Ay(2, nb_cont),        Az(3, nb_cont),       ///< matrices for force distribution	
	Ax_trans(nb_cont, 2),  Ay_trans(nb_cont, 2),  Az_trans(nb_cont, 3), ///< transpose matrices
	Ax_mult(2, 2),         Ay_mult(2, 2),         Az_mult(3, 3),        ///< multiplication matrices
	Ax_mult_inv(2, 2),     Ay_mult_inv(2, 2),     Az_mult_inv(3, 3),    ///< inverse matrices
	Ax_ps_inv(nb_cont, 2), Ay_ps_inv(nb_cont, 2), Az_ps_inv(nb_cont, 3) ///< pseudo inverse matrices
{
	this->nb_cont = nb_cont;
	this->fwd_kin = fwd_kin;
	this->nb_mot  = nb_mot;
	this->outputs = outputs;

	io = fwd_kin->get_io_ptr();

	// safety
	if (nb_mot != io->get_nb_mot())
	{
		std::cout << "Error: nb_mot (" << nb_mot << ") should be " << io->get_nb_mot() << " !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// fill last line with 1.0
	for(int i=0; i<nb_cont; i++)
	{
		Ax.set(1, i, 1.0);
		Ay.set(1, i, 1.0);
		Az.set(2, i, 1.0);
	}

	// COM position reference
	r_COM_ref[0] = 0.0;
	r_COM_ref[1] = 0.0;
	r_COM_ref[2] = 0.56;

	// COM tracking gains
	kp_x = 110.0;
	kp_y = 110.0;
	kp_z = 2200.0;

	kd_x = 50.0;
	kd_y = 50.0;
	kd_z = 50.0;

	ki_x = 0.0;
	ki_y = 0.0;
	ki_z = 0.0;

	// basic torque tracking gains
	kp_trq = 1.0;
	kd_trq = 1.0;

	// maximal torque
	max_tor = 30.0;

	// prevent knee over-extension
	thres_knee = 0.01;
	thres_knee_gain = 100.0;

	for(int i=0; i<3; i++)
	{
		fp[i] = 0.0;

		r_base[i]  = 0.0;
		rp_base[i] = 0.0;

		delta_com[i]     = 0.0;
		delta_com_dot[i] = 0.0;
		delta_com_int[i] = 0.0;
	}

	last_t_delta_com = inputs->get_t();

	for(int i=0; i<2; i++)
	{
		rp[i] = 0.0;
	}

	// joint IDs
	RightKneePitch_ctrl_id = ctrl_index->get_inv_index(CtrlIndex::RightKneePitch);
	LeftKneePitch_ctrl_id  = ctrl_index->get_inv_index(CtrlIndex::LeftKneePitch);

	RightKneePitch_fwd_kin_id = fwd_kin->get_fwd_kin_index(CtrlIndex::RightKneePitch);
	LeftKneePitch_fwd_kin_id  = fwd_kin->get_fwd_kin_index(CtrlIndex::LeftKneePitch);

	// references
	for(int i=0; i<nb_mot; i++)
	{
		trq_ref.push_back(0.0);
		pos_ref.push_back(0.0);

		ctrl_id.push_back(fwd_kin->get_ctrl_index(i));
		q_ctrl_id.push_back(ctrl_index->get_inv_index(ctrl_id[i]));
	}
}

/*! \brief destructor
 */
ImpedanceCtrl::~ImpedanceCtrl()
{

}

/*! \brief main computation of the impedance-based controller
 */
void ImpedanceCtrl::compute()
{
	// base position and time derivative
	compute_base();

	// Ax, Ay, Az matrices fill
	update_matrices();

	// Ax, Ay, Az pseudo-inverse
	inverse_matrices();

	// ground applied forces
	compute_GAF();

	// force distribution
	compute_force_distrib();

	// jacobian components
	compute_jacob();

	// compute torques
	compute_tor();

	// set the torque references in the inputs class (used later by other modules)
	set_input_Qq_ref();
}

/*! \brief set the COM position reference
 * 
 * \param[in] x x position of the COM (relative to the waist) [m]
 * \param[in] y y position of the COM (relative to the waist) [m]
 * \param[in] z z position of the COM (relative to the waist) [m]
 */
void ImpedanceCtrl::set_r_COM_ref(double x, double y, double z)
{
	r_COM_ref[0] = x;
	r_COM_ref[1] = y;
	r_COM_ref[2] = z;
}

/*! \brief update the pseudo-inverses of the Ax, Ay, Az matrices
 */
void ImpedanceCtrl::inverse_matrices()
{
	// compute pseudo-inverses
	if (!pseudo_inverse_matrix(Ax, Ax_trans, Ax_mult, Ax_mult_inv, Ax_ps_inv) ||
		!pseudo_inverse_matrix(Ay, Ay_trans, Ay_mult, Ay_mult_inv, Ay_ps_inv) ||
		!pseudo_inverse_matrix(Az, Az_trans, Az_mult, Az_mult_inv, Az_ps_inv) )
	{
		if (options->is_print())
		{
			std::cout << "Warning: singular matrix during pseude matrix inversion for impedance-based controller !" << std::endl;
		}
		return;
	}
}

/*! \brief compute the ground applied forces
 */
void ImpedanceCtrl::compute_GAF()
{
	double delta_t;

	// COM relative to the base
	for(int i=0; i<3; i++)
	{
		delta_com[i]     = io->r_COM[i]  - r_base[i];
		delta_com_dot[i] = io->rp_COM[i] - rp_base[i];		
	}

	// error integral
	delta_t = inputs->get_t() - last_t_delta_com;

	if (delta_t > 0.0)
	{
		last_t_delta_com = inputs->get_t();

		for(int i=0; i<3; i++)
		{
			delta_com_int[i] += delta_t * delta_com[i];
		}
	}

	// COM tracking
	fp[0] =  kp_x * (r_COM_ref[0] - delta_com[0]) - kd_x * delta_com_dot[0] - ki_x * delta_com_int[0];
	fp[1] = -kp_y * (r_COM_ref[1] - delta_com[1]) + kd_y * delta_com_dot[1] + ki_y * delta_com_int[1];	
	fp[2] = -kp_z * (r_COM_ref[2] - delta_com[2]) + kd_z * delta_com_dot[2] + ki_z * delta_com_int[2];

	// add gravity compensation
	fp[2] -= WEIGHT_COMAN;
}

/*! \brief compute the force distribution
 */
void ImpedanceCtrl::compute_force_distrib()
{
	// desired COP position (x and y)
	rp[0] = (delta_com[2] * fp[0]) / fp[2];
	rp[1] = (delta_com[2] * fp[1]) / fp[2];

	// individual contact forces
	for(int i=0; i<nb_cont; i++)
	{
		fs.set(0, i, Ax_ps_inv.get(i,0)*fp[0]*rp[0] + Ax_ps_inv.get(i,1)*fp[0]);
		fs.set(1, i, Ay_ps_inv.get(i,0)*fp[1]*rp[1] + Ay_ps_inv.get(i,1)*fp[1]);
		fs.set(2, i, Az_ps_inv.get(i,0)*fp[2]*rp[0] + Az_ps_inv.get(i,1)*fp[2]*rp[1] + Az_ps_inv.get(i,2)*fp[2]);
	}
}

/*! \brief compute the joint torques
 */
void ImpedanceCtrl::compute_tor()
{
	for(int i=0; i<nb_mot; i++)
	{
		trq_ref[i] = kp_trq * (pos_ref[i] - inputs->get_q_mot(q_ctrl_id[i])) - kd_trq * inputs->get_qd_mot(q_ctrl_id[i]);

		for (int j=0; j<nb_cont; j++)
		{
			trq_ref[i]  += fs.get(0,j) * jacob_x.get(i,j)
						+  fs.get(1,j) * jacob_y.get(i,j)
						+  fs.get(2,j) * jacob_z.get(i,j);
		}
	}

	// prevent knee over-extension
	if(inputs->get_q_mot(RightKneePitch_ctrl_id) < thres_knee)
	{
		trq_ref[RightKneePitch_fwd_kin_id] += thres_knee_gain * (thres_knee - inputs->get_q_mot(RightKneePitch_ctrl_id));
	}

	if(inputs->get_q_mot(LeftKneePitch_ctrl_id) < thres_knee)
	{
		trq_ref[LeftKneePitch_fwd_kin_id] += thres_knee_gain * (thres_knee - inputs->get_q_mot(LeftKneePitch_ctrl_id));
	}

	// limit torque
	for(int i=0; i<nb_mot; i++)
	{
		trq_ref[i] = limit(trq_ref[i], -max_tor, max_tor);
	}
}

/*! \brief send the torque references to the joint outputs
 */
void ImpedanceCtrl::apply_Qq_ref()
{
	for(int i=0; i<nb_mot; i++)
	{
		outputs->set_Qq_ref(q_ctrl_id[i], trq_ref[i]);
	}
}

/*! \brief send the torque references to the joint inputs (to be used by other modules)
 */
void ImpedanceCtrl::set_input_Qq_ref()
{
	for(int i=0; i<nb_mot; i++)
	{
		inputs->set_Qq_ref(q_ctrl_id[i], trq_ref[i]);
	}
}
