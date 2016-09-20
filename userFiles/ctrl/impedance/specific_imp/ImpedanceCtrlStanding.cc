#include "ImpedanceCtrlStanding.hh"

/*! \brief constructor
 *
 * \param[in] inputs controller inputs
 * \param[in] options controller options
 * \param[in] ctrl_index controller index lists
 * \param[in] outputs controller outputs
 * \param[in] fwd_kin forward kinematics
 */
ImpedanceCtrlStanding::ImpedanceCtrlStanding(CtrlInputs *inputs, CtrlOptions *options, MotorCtrlIndex *ctrl_index, CtrlOutputs *outputs, ForwardKinematics *fwd_kin):
	ImpedanceCtrl(inputs, options, ctrl_index, outputs, fwd_kin, 8, fwd_kin->get_io_ptr()->get_nb_mot())
{
	nb_cont_foot = 4;
}

/*! \brief destructor
 */
ImpedanceCtrlStanding::~ImpedanceCtrlStanding()
{
	
}

/*! \brief compute the base position and time derivative
 */
void ImpedanceCtrlStanding::compute_base()
{
	for(int i=0; i<3; i++)
	{
		r_base[i]  = 0.5 * (io->r_Rfoot[i]  + io->r_Lfoot[i] );
		rp_base[i] = 0.5 * (io->rp_Rfoot[i] + io->rp_Lfoot[i]);
	}
}

/*! \brief update the Ax, Ay, Az matrices and their pseudo-inverses
 */
void ImpedanceCtrlStanding::update_matrices()
{
	// fill matrices
	for(int i=0; i<nb_cont_foot; i++)
	{
		// right foot
		Ax.set(0, i, io->r_Rfoot_cont[i][0] - io->r_COM[0]);
		Ay.set(0, i, io->r_Rfoot_cont[i][1] - io->r_COM[1]);

		Az.set(0, i, io->r_Rfoot_cont[i][0] - io->r_COM[0]);
		Az.set(1, i, io->r_Rfoot_cont[i][1] - io->r_COM[1]);

		// left foot
		Ax.set(0, i+nb_cont_foot, io->r_Lfoot_cont[i][0] - io->r_COM[0]);
		Ay.set(0, i+nb_cont_foot, io->r_Lfoot_cont[i][1] - io->r_COM[1]);

		Az.set(0, i+nb_cont_foot, io->r_Lfoot_cont[i][0] - io->r_COM[0]);
		Az.set(1, i+nb_cont_foot, io->r_Lfoot_cont[i][1] - io->r_COM[1]);
	}
}

/*! \brief compute the jacobian components
 */
void ImpedanceCtrlStanding::compute_jacob()
{
	for(int i=0; i<nb_mot; i++)
	{
		for (int j=0; j<nb_cont_foot; j++)
		{
			// right foot
			jacob_x.set(i, j, io->r_Rfoot_cont_der[j][i][0] - io->r_COM_der[i][0]);
			jacob_y.set(i, j, io->r_Rfoot_cont_der[j][i][1] - io->r_COM_der[i][1]);
			jacob_z.set(i, j, io->r_Rfoot_cont_der[j][i][2] - io->r_COM_der[i][2]);

			// left foot
			jacob_x.set(i, j+nb_cont_foot, io->r_Lfoot_cont_der[j][i][0] - io->r_COM_der[i][0]);
			jacob_y.set(i, j+nb_cont_foot, io->r_Lfoot_cont_der[j][i][1] - io->r_COM_der[i][1]);
			jacob_z.set(i, j+nb_cont_foot, io->r_Lfoot_cont_der[j][i][2] - io->r_COM_der[i][2]);
		}
	}
}
