#include "Matrix2D.hh"
#include <cmath>

/// get square value
inline double square(double x) { return x*x; }

/*! \brief constructor
 * 
 * \param[in] m first array dimension
 * \param[in] n second array dimension
 */
Matrix2D::Matrix2D(int m, int n): Array2D<double>(m, n)
{

}

/*! \brief destructor
 */
Matrix2D::~Matrix2D()
{

}

/*! \brief set the matrix to this identity form
 *
 * \pre matrix must be square
 */
void Matrix2D::identity()
{
	// safety
	if (get_m() != get_n())
	{
		std::cout << "Error: identity matrix only for square matrices, not for " << get_m() << "x" << get_n() << " !" << std::endl;
		exit(EXIT_FAILURE);
	}

	reset();

	for(int i=0; i<get_n(); i++)
	{
		set(i, i, 1.0);
	}
}

/*! \brief update the transpose matrix
 *
 * \param[out] transpose_matrix matrix to transpose
 */
void Matrix2D::transpose(Matrix2D &transpose_matrix) const
{
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			transpose_matrix.set(j, i, get(i, j));
		}
	}
}

/*! \brief post-multiply a matrix
 * 
 * \param[in] other_matrix matrix to post-multiply
 * \param[out] result_matrix result of the matrix multiplication
 */
void Matrix2D::post_multiply_matrix(Matrix2D const& other_matrix, Matrix2D &result_matrix) const
{
	double cur_sum;

	// safety 1
	if (get_n() != other_matrix.get_m())
	{
		std::cout << "Multiplicatoin error: matrix A is " << get_m() << "x" << get_n()
			<< " and matrix B is " << other_matrix.get_m() << "x" << other_matrix.get_n() << " !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// safety 2
	if (result_matrix.get_m() != get_m() || result_matrix.get_n() != other_matrix.get_n())
	{
		std::cout << "Multiplicatoin error: matrix A is " << get_m() << "x" << get_n()
			<< ", matrix B is " << other_matrix.get_m() << "x" << other_matrix.get_n() 
			<< " and result matrix is " << result_matrix.get_m() << "x" << result_matrix.get_n() << " !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// multilication
	for(int i=0; i<result_matrix.get_m(); i++)
	{
		for(int j=0; j<result_matrix.get_n(); j++)
		{
			cur_sum = 0.0;

			for(int k=0; k<get_n(); k++)
			{
				cur_sum += get(i, k) * other_matrix.get(k, j);
			}

			result_matrix.set(i, j, cur_sum);
		}
	}
}

/*! \brief pre-multiply a matrix
 * 
 * \param[in] other_matrix matrix to pre-multiply
 * \param[out] result_matrix result of the matrix multiplication
 */
void Matrix2D::pre_multiply_matrix(Matrix2D const& other_matrix, Matrix2D &result_matrix) const
{
	other_matrix.post_multiply_matrix(*this, result_matrix);
}

/*! \brief get the square of the norm of the matrix
 */
double Matrix2D::get_square_norm()
{
	double cur_sum;

	cur_sum = 0.0;

	for(int i=0; i<m; i++)
	{
		for(int j=0; j<n; j++)
		{
			cur_sum += square(get(i, j));
		}
	}

	return cur_sum;
}

/*! \brief inverse a square matrix using Gaussian elimination (matrix is modified)
 * 
 * \pre all matrices must be square matrix of the same size
 * \param[in] matrix matrix to inverse
 * \param[out] result result matrix
 * \return 1 if inversion succeeded, 0 otherwise (singular matrix)
 */
int inverse_square_matrix(Matrix2D &matrix, Matrix2D &result)
{
	int n, pivot;
	double pivot_val, cur_val;

	n = matrix.get_n();
	
	// safety
	if (matrix.get_m() != n || result.get_m() != n || result.get_n() != n)
	{
		std::cout << "Matrix inversion error: error all matrices should be square with the same size ! " << std::endl;
		exit(EXIT_FAILURE);
	}

	// result initialized to identity matrix
	result.identity();


	// -- Gaussian elimination -- //

	// loop on all the columns
	for(int i=0; i<n; i++)
	{
		// find pivot
		pivot = i;
		pivot_val = fabs(matrix.get(i, i));

		for(int j=i+1; j<n; j++)
		{
			cur_val = fabs(matrix.get(j, i));
			if (cur_val > pivot_val)
			{
				pivot_val = cur_val;
				pivot = j;
			}
		}

		// swap rows
		if (pivot != i)
		{
			matrix.swap_rows(i, pivot);
			result.swap_rows(i, pivot);
		}

		// remove abs
		pivot_val = matrix.get(i, i);

		// singular matrix
		if (!pivot_val)
		{
			result.reset();
			return 0;
		}

		// loop on all rows below pivot
		for(int j=i+1; j<n; j++)
		{
			cur_val = matrix.get(j, i) / pivot_val;

			// loop on all remaining columns of current row
			for(int k=i+1; k<n; k++)
			{
				matrix.set(j, k, matrix.get(j, k) - matrix.get(i, k)*cur_val);
			}

			// apply the same rule on the result matrix
			for(int k=0; k<n; k++)
			{
				result.set(j, k, result.get(j, k) - result.get(i, k)*cur_val);
			}

			// fill lower triangle with 0
			matrix.set(j, i, 0.0);
		}
	}

	// loop on all the columns (inverse order)
	for(int i=n-1; i>=0; i--)
	{
		// pivot value
		pivot_val = matrix.get(i, i);

		// loop on all rows above pivot
		for(int j=i-1; j>=0; j--)
		{
			cur_val = matrix.get(j, i) / pivot_val;

			// loop on all remaining columns of current row
			for(int k=i-1; k>=0; k--)
			{
				matrix.set(j, k, matrix.get(j, k) - matrix.get(i, k)*cur_val);
			}

			// apply the same rule on the result matrix
			for(int k=n-1; k>=0; k--)
			{
				result.set(j, k, result.get(j, k) - result.get(i, k)*cur_val);
			}

			// fill upper triangle with 0
			matrix.set(j, i, 0.0);
		}
	}

	// loop on all the rows
	for(int i=0; i<n; i++)
	{
		// pivot value
		pivot_val = matrix.get(i, i);

		// loop on all the columns
		for(int j=0; j<n; j++)
		{
			matrix.set(i, j, matrix.get(i, j) / pivot_val);
			result.set(i, j, result.get(i, j) / pivot_val);
		}
	}

	// inversion succeeded
	return 1;
}

/*! \brief inverse a square matrix using Gaussian elimination (matrix is not modified)
 * 
 * \pre all matrices must be square matrix of the same size
 * \param[in] matrix matrix to inverse
 * \param[out] copy matrix of the same size as 'matrix', used to perform a copy, and then to modify it
 * \param[out] result result matrix
 * \return 1 if inversion succeeded, 0 otherwise (singular matrix)
 */
int inverse_square_matrix(Matrix2D const& matrix, Matrix2D &copy, Matrix2D &result)
{
	// safety
	if (copy.get_m() != matrix.get_m() || copy.get_n() != matrix.get_n())
	{
		std::cout << "Matrix inversion error: 'matrix' and 'copy' should be the same size ! " << std::endl;
		exit(EXIT_FAILURE);
	}

	// copy 'matrix' in 'copy'
	copy.copy(matrix);

	return inverse_square_matrix(copy, result);
}

/*! \brief compute the pseudo-inverse of a non-square (or square) matrix
 * 
 * \param[in] matrix matrix to inverse
 * \param[out] trans transpose of the matrix
 * \param[out] mult matrix*trans
 * \param[out] mult_inv inverse of 'mult'
 * \param[out] result requested pseudo inverse matrix
 * \return 1 if successful, 0 otherwise (singular square matrix)
 */
int pseudo_inverse_matrix(Matrix2D const& matrix, Matrix2D &trans, Matrix2D &mult, Matrix2D &mult_inv, Matrix2D &result)
{
	int m, n;

	m = matrix.get_m();
	n = matrix.get_n();

	// safety
	if (trans.get_m()    != n || trans.get_n()    != m ||
		mult.get_m()     != m || mult.get_n()     != m ||
		mult_inv.get_m() != m || mult_inv.get_n() != m ||
		result.get_m()   != n || result.get_n()   != m )
	{
		std::cout << "Error with matrix sizes when computing the pseudo inverse !" << std::endl;
		exit(EXIT_FAILURE);
	}

	// transpose matrix
	matrix.transpose(trans);

	// multiplication
	matrix.post_multiply_matrix(trans, mult);

	// inverse of the multiplication
	if (!inverse_square_matrix(mult, mult_inv))
	{
		// singular matrix during inversion
		result.reset();
		return 0;
	}

	// pseudo inverse
	trans.post_multiply_matrix(mult_inv, result);

	// pseudo inverse successfully computed
	return 1;
}

/*! \brief sum two matrices
 * 
 * \param[in] A first matrix to sum
 * \param[in] B second matrix to sum
 * \param[in] result result of the summation
 */
void sum_matrix(Matrix2D const& A, Matrix2D const& B, Matrix2D &result)
{
	// safety
	if (A.get_m() != B.get_m() || A.get_n() != B.get_n() )
	{
		std::cout << "Error: matrices must be the same size for summation !" << std::endl;
		exit(EXIT_FAILURE);
	}

	for(int i=0; i<A.get_m(); i++)
	{
		for(int j=0; j<A.get_n(); j++)
		{
			result.set(i, j, A.get(i, j) + B.get(i, j));
		}
	}
}

/*! \brief difference of two matrices
 * 
 * \param[in] A first matrix
 * \param[in] B second matrix
 * \param[in] result result of the difference
 */
void diff_matrix(Matrix2D const& A, Matrix2D const& B, Matrix2D &result)
{
	// safety
	if (A.get_m() != B.get_m() || A.get_n() != B.get_n() )
	{
		std::cout << "Error: matrices must be the same size for summation !" << std::endl;
		exit(EXIT_FAILURE);
	}

	for(int i=0; i<A.get_m(); i++)
	{
		for(int j=0; j<A.get_n(); j++)
		{
			result.set(i, j, A.get(i, j) - B.get(i, j));
		}
	}
}
