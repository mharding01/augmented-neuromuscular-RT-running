#include "opti_params.hh"
#include <iostream>
#include <cstdlib>

/*! \brief get the optimized parameters exact values
 * 
 * \param[in] optiNorms normalized optimization parameters
 * \return real parameter corresponding to the last normalized parameter added
 */
double convert_to_optiParams(std::vector<double> optiNorms)
{
	int index;
	double last_elem;

	index = optiNorms.size() - 1;

	if (index < 0)
	{
		std::cout << "Error: optiNorms is empty !" << std::endl;
		exit(EXIT_FAILURE);
	}

	last_elem = optiNorms[index];

	switch (index)
	{
		case 0 : return (last_elem * 0.12000000 + 1.48000000);
		case 1 : return (last_elem * 0.20000000 + 2.35000000);
		case 2 : return (last_elem * 0.46000000 + 2.55000000);
		case 3 : return (last_elem * 0.08000000 + 5.45000000);
		case 4 : return (last_elem * 0.10000000 + 4.80000000);
		case 5 : return (last_elem * 0.38000000 + 5.62000000);
		case 6 : return (last_elem * 0.05000000 + 3.73000000);
		case 7 : return (last_elem * 0.08000000 + 3.64000000);
		case 8 : return (last_elem * 4.00000000 + -2.00000000);
		case 9 : return (last_elem * 4.00000000 + -2.00000000);
		case 10 : return (last_elem * 4.00000000 + -2.00000000);
		case 11 : return (last_elem * 4.00000000 + -2.00000000);
		case 12 : return (last_elem * 4.00000000 + -2.00000000);
		case 13 : return (last_elem * 4.00000000 + -2.00000000);
		case 14 : return (last_elem * 4.00000000 + -2.00000000);
		case 15 : return (last_elem * 4.00000000 + -2.00000000);
		case 16 : return (last_elem * 4.00000000 + -2.00000000);
		case 17 : return (last_elem * 4.00000000 + -2.00000000);
		case 18 : return (last_elem * 4.00000000 + -2.00000000);
		case 19 : return (last_elem * 4.00000000 + -2.00000000);
		case 20 : return (last_elem * 4.00000000 + -2.00000000);
		case 21 : return (last_elem * 4.00000000 + -2.00000000);
		case 22 : return (last_elem * 2.00000000 + -1.00000000);
		case 23 : return (last_elem * 6.00000000 + -3.00000000);
		case 24 : return (last_elem * 20.00000000 + -10.00000000);
		case 25 : return (last_elem * 2.00000000 + -1.00000000);
		case 26 : return (last_elem * 6.00000000 + -3.00000000);
		case 27 : return (last_elem * 20.00000000 + -10.00000000);
		case 28 : return (last_elem * 2.00000000 + -1.00000000);
		case 29 : return (last_elem * 6.00000000 + -3.00000000);
		case 30 : return (last_elem * 20.00000000 + -10.00000000);
		case 31 : return (last_elem * 0.04000000 + 0.01000000);
		case 32 : return (last_elem * 0.04000000 + 0.01000000);
		case 33 : return (last_elem * 0.04000000 + 0.01000000);
		case 34 : return (last_elem * 0.66300000 + 0.04700000);
		case 35 : return (last_elem * 0.31000000 + 0.01000000);
		case 36 : return (last_elem * 0.49000000 + 0.01000000);
		case 37 : return (last_elem * 0.31000000 + 0.01000000);
		case 38 : return (last_elem * 0.31000000 + 0.01000000);
		case 39 : return (last_elem * 0.04000000 + 0.01000000);
		case 40 : return (last_elem * 0.04000000 + 0.01000000);
		case 41 : return (last_elem * 0.19000000 + 0.01000000);
		case 42 : return (last_elem * 0.09000000 + 0.01000000);
		case 43 : return (last_elem * 0.09000000 + 0.01000000);
		case 44 : return (last_elem * 0.49000000 + 0.01000000);
		case 45 : return (last_elem * 0.04000000 + 0.01000000);
		case 46 : return (last_elem * 0.04000000 + 0.01000000);
		case 47 : return (last_elem * 3.00000000 + 0.00000000);
		case 48 : return (last_elem * 2.00000000 + 0.00000000);
		case 49 : return (last_elem * 4.45000000 + 0.55000000);
		case 50 : return (last_elem * 4.45000000 + 0.55000000);
		case 51 : return (last_elem * 2.83000000 + 0.17000000);
		case 52 : return (last_elem * 10.00000000 + 0.00000000);
		case 53 : return (last_elem * 0.30000000 + 0.50000000);
		case 54 : return (last_elem * 0.30000000 + 0.50000000);
		case 55 : return (last_elem * 0.71000000 + 0.00000000);
		case 56 : return (last_elem * 0.85000000 + 0.00000000);
		case 57 : return (last_elem * 5.50000000 + 0.50000000);
		case 58 : return (last_elem * 13.50000000 + 0.50000000);
		case 59 : return (last_elem * 9.00000000 + 1.00000000);
		case 60 : return (last_elem * 0.50000000 + 0.00000000);
		case 61 : return (last_elem * 0.50000000 + 0.00000000);
		case 62 : return (last_elem * 0.80000000 + 0.00000000);
		case 63 : return (last_elem * 0.70000000 + 0.30000000);
		case 64 : return (last_elem * 1.00000000 + 0.00000000);
		case 65 : return (last_elem * 0.80000000 + 0.20000000);
		case 66 : return (last_elem * 1.00000000 + 0.00000000);
		case 67 : return (last_elem * 5.00000000 + 0.00000000);
		case 68 : return (last_elem * 5.00000000 + 0.00000000);
		case 69 : return (last_elem * 5.00000000 + 0.00000000);
		case 70 : return (last_elem * 0.10000000 + 0.00000000);
		case 71 : return (last_elem * 0.10000000 + 0.00000000);
		case 72 : return (last_elem * 0.10000000 + 0.00000000);
		case 73 : return (last_elem * 0.30000000 + 0.00000000);
		case 74 : return (last_elem * 0.40000000 + -0.30000000);
		case 75 : return (last_elem * 0.50000000 + 0.20000000);
		case 76 : return (last_elem * 4.70000000 + 1.00000000);
		case 77 : return (last_elem * 0.15000000 + 0.05000000);
		case 78 : return (last_elem * 0.12000000 + 0.46000000);
		case 79 : return (last_elem * 1.00000000 + -0.50000000);
		case 80 : return (last_elem * 0.30000000 + 0.00000000);
		case 81 : return (last_elem * 10.00000000 + -5.00000000);
		case 82 : return (last_elem * 10.00000000 + -5.00000000);
		case 83 : return (last_elem * 0.52000000 + -0.87000000);
		case 84 : return (last_elem * 10.00000000 + -5.00000000);
		case 85 : return (last_elem * 0.87000000 + 0.00000000);
		case 86 : return (last_elem * 10.00000000 + -5.00000000);
		case 87 : return (last_elem * 0.18000000 + -0.09000000);
		case 88 : return (last_elem * 10.00000000 + -5.00000000);
		case 89 : return (last_elem * 0.44000000 + -0.09000000);
		case 90 : return (last_elem * 10.00000000 + -5.00000000);
		case 91 : return (last_elem * 1.40000000 + 0.35000000);
		case 92 : return (last_elem * 10.00000000 + -5.00000000);
		case 93 : return (last_elem * 0.36000000 + 0.09000000);
		case 94 : return (last_elem * 10.00000000 + -5.00000000);

		default:
			std::cout << "Error: optiNorms is too small: " << index << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
}

/*! \brief get the number of optimized parameters
 * 
 * \return number of optimized parameters
 */
int get_nb_optiParams()
{
	return 95;
}
