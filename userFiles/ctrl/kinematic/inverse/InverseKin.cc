#include "InverseKin.hh"
#include "coman_properties.hh"

/*! \brief constructor
 *
 * \param[in] fwd_kin forward kinematics module
 * \param[in] nb_const number of constraints to solve
 */
InverseKin::InverseKin(ForwardKinematics *fwd_kin, CtrlInputs *inputs, int nb_const): io(NB_MOTORS), h(nb_const, 1), pos(nb_const, 1),
	delta_pos(nb_const, 1), jac(nb_const, nb_const), jac_inv(nb_const, nb_const)
{
	this->fwd_kin  = fwd_kin;
	this->inputs   = inputs;
	this->nb_const = nb_const;

	// initialize position to 0
	for(int i=0; i<nb_const; i++)
	{
		pos.set(i, 0, 0.0);
	}

	nb_max_iter = 20;

	h_norm_thres = 1.0e-20;
}

/*! \brief destructor
 */
InverseKin::~InverseKin()
{

}

/*! \brief inverse kinematics main computation
 *
 * \return 1 if inverse kinematics successfully computed, 0 oterwise
 */
int InverseKin::compute()
{
	int flag_succeed;

	fill_init_pos();

	flag_succeed = 0;

	for(int i=0; i<nb_max_iter; i++)
	{
		fill_pos();

		compute_iteration();

		// convergence achieved
		if (h.get_square_norm() < h_norm_thres)
		{
			flag_succeed = 1;
			break;
		}
	}

	// convergence not achieved
	if (!flag_succeed)
	{
		fill_init_pos();
		return 0;
	}
	else
	{
		return 1;
	}
}

/*! \brief inverse kinematics: single iteration computation
 */
void InverseKin::compute_iteration()
{
	// main kinematics computation
	fwd_kin->main_kinematics(io);

	// fill constraints and jacobian
	fill_constraints();
	fill_jacobian();

	// inverse matrix
	if (inverse_square_matrix(jac, jac_inv))
	{
		// increment matrix position
		jac_inv.post_multiply_matrix(h, delta_pos);
		diff_matrix(pos, delta_pos, pos);
	}
}
