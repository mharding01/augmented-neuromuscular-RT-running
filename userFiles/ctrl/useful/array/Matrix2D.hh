/*! 
 * \author Nicolas Van der Noot
 * \file Matrix2D.hh
 * \brief Matrix2D class
 */

#ifndef _MATRIX_2D_HH_
#define _MATRIX_2D_HH_

#include "Array2D.hh"

class Matrix2D: public Array2D<double>
{
	public:
		Matrix2D(int m, int n);
		virtual ~Matrix2D();

		void identity();

		void transpose(Matrix2D &transpose_matrix) const;

		void post_multiply_matrix(Matrix2D const& other_matrix, Matrix2D &result_matrix) const;
		void pre_multiply_matrix(Matrix2D const& other_matrix, Matrix2D &result_matrix) const;

		double get_square_norm();

		/// set all values to 0
		void reset()
		{
			for(int i=0; i<m; i++)
			{
				for(int j=0; j<n; j++)
				{
					set(i, j, 0.0);
				}
			}
		}
};

// function prototypes
int inverse_square_matrix(Matrix2D &matrix, Matrix2D &result);
int inverse_square_matrix(Matrix2D const& matrix, Matrix2D &copy, Matrix2D &result);
int pseudo_inverse_matrix(Matrix2D const& matrix, Matrix2D &trans, Matrix2D &mult, Matrix2D &mult_inv, Matrix2D &result);

void sum_matrix(Matrix2D const& A, Matrix2D const& B, Matrix2D &result);
void diff_matrix(Matrix2D const& A, Matrix2D const& B, Matrix2D &result);

#endif
