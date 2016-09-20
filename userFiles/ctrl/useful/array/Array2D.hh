/*! 
 * \author Nicolas Van der Noot
 * \file Array2D.hh
 * \brief Array2D class
 */

#ifndef _ARRAY_2D_HH_
#define _ARRAY_2D_HH_

#include <iostream>
#include <stdlib.h>

/*! \brief 2D class with template for a 2D array
 */
template <class T>
class Array2D
{
	public:
		/*! \brief constructor
		 * 
		 * \param[in] m first array dimension
		 * \param[in] n second array dimension
		 */
		Array2D(int m, int n)
		{
			this->m = m;
			this->n = n;
			size = m*n;

			flag_square = (m == n) ? 1 : 0;

			if (m >= 1 && n >= 1)
			{
				tab_ptr = new T[m*n];
			}
			else
			{
				std::cout << "Error: 2D array arguments must be at least 1 !" << std::endl;
				exit(EXIT_FAILURE);
			}
		}

		/*! \brief destructor
		 */
		virtual ~Array2D()
		{
			delete[] tab_ptr;
		}

		/*! \brief copy other matrix elements
		 * 
		 * \pre other matrix must be the same size
		 * \param[in] other other matrix to copy
		 */
		void copy(Array2D const& other)
		{
			// saftety
			if (m != other.get_m() || n != other.get_n())
			{
				std::cout << "Error during copy: not same matrix sizes !" << std::endl;
				exit(EXIT_FAILURE);
			}

			// copy
			for(int i=0; i<size; i++)
			{
				tab_ptr[i] = other.get_vec(i);
			}
		}

		/*! \brief paste this matrix content to an other one
		 * 
		 * \pre other matrix must be the same size
		 * \param[in] other other matrix to paste
		 */
		void paste(Array2D &other) const
		{
			// saftety
			if (m != other.get_m() || n != other.get_n())
			{
				std::cout << "Error during copy: not same matrix sizes !" << std::endl;
				exit(EXIT_FAILURE);
			}

			// copy
			for(int i=0; i<size; i++)
			{
				other.set_vec(i, tab_ptr[i]);
			}
		}

		/*! \brief swap two rows
		 * 
		 * \param[in] x first row index
		 * \param[in] y second row index
		 */
		void swap_rows(int x, int y)
		{
			double cur_copy;

			for(int i=0; i<n; i++)
			{
				cur_copy = get(x, i);

				set(x, i, get(y, i));
				set(y, i, cur_copy);
			}
		}

		/*! \brief swap two columns
		 * 
		 * \param[in] x first column index
		 * \param[in] y second column index
		 */
		void swap_columns(int x, int y)
		{
			double cur_copy;

			for(int i=0; i<m; i++)
			{
				cur_copy = get(i, x);

				set(i, x, get(i, y));
				set(i, y, cur_copy);
			}
		}

		/*! \brief print the matrix
		 */
		void print()
		{
			for(int i=0; i<m; i++)
			{
				for(int j=0; j<n; j++)
				{
					if (!j)
					{
						std::cout << "[ ";
					}
					else
					{
						std::cout << " ; ";
					}
					std::cout << get(i, j);
					if (j == n-1)
					{
						std::cout << " ]";
					}
				}
				std::cout << std::endl;
			}
		}

		/// get value at index [i] (like 1D vector)
		T& get_vec(int i) const { return tab_ptr[i]; }

		/// get copy value at index [i] (like 1D vector)
		T get_vec_copy(int i) const { return tab_ptr[i]; }

		/// get value at index [x;y]
		T& get(int x, int y) const { return tab_ptr[x*n + y]; }

		/// get copy value at index [x;y]
		T get_copy(int x, int y) const { return tab_ptr[x*n + y]; }

		/// set value at index [x;y]
		void set(int x, int y, T value) { tab_ptr[x*n + y] = value; }

		/// set value at index [i] (like 1D vector)
		void set_vec(int i, T value) { tab_ptr[i] = value; }

		/// get first array dimension
		int get_m() const { return m; }

		/// get second array dimension
		int get_n() const { return n; }

	protected:
		int m; ///< first array dimension
		int n; ///< second array dimension
		int size; ///< vector 1D size: m*n
		int flag_square; ///< 1 if square array, 0 otherwise

		T *tab_ptr; ///< pointer to a one dimensional tabular for 2D array
};

#endif
