/*! 
 * \author Nicolas Van der Noot
 * \file InverseKin.hh
 * \brief InverseKin class
 */

#ifndef _INVERSE_KIN_HH_
#define _INVERSE_KIN_HH_

#include "ForwardKinematics.hh"
#include "Matrix2D.hh"
#include <cmath>

/*! \brief inverse kinematics module computation
 */
class InverseKin
{
	public:
		InverseKin(ForwardKinematics *fwd_kin, CtrlInputs *inputs, int nb_const);
		virtual ~InverseKin();

		int compute();

		virtual void fill_pos() = 0;
		virtual void fill_init_pos() = 0;
		virtual void fill_constraints() = 0;
		virtual void fill_jacobian() = 0;

		void set_nb_max_iter(int value) { nb_max_iter = value; }
		void set_h_norm_thres(double value) { h_norm_thres = value; }

		KinematicsInOut& get_io() { return io; }
		Matrix2D& get_pos() { return pos; }

	protected:
		int nb_const; ///< number of constraints
		int nb_max_iter; ///< maximal number of iterations

		double h_norm_thres; ///< norm of h threshold to stop iteration

		ForwardKinematics *fwd_kin; ///< forward kinematics
		CtrlInputs *inputs; ///< controller inputs

		KinematicsInOut io; ///< I/O of the kinematics model

		Matrix2D h; ///< constraints to solve

		Matrix2D jac;      ///< Jacobian of the constraint
		Matrix2D jac_inv;  ///< inverse of the jacobian

		Matrix2D pos;       ///< position vector (result)
		Matrix2D delta_pos; ///< increment of 'pos'

		// function prototype
		void compute_iteration();
};

#endif
