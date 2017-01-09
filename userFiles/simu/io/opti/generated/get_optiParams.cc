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
		case 0 : return (last_elem * 0.14000000 + 0.01000000);
		case 1 : return (last_elem * 0.25000000 + 0.00000000);
		case 2 : return (last_elem * 0.85000000 + 0.05000000);
		case 3 : return (last_elem * 5.80000000 + 0.30000000);
		case 4 : return (last_elem * 9.00000000 + 1.00000000);
		case 5 : return (last_elem * 6.70000000 + 0.30000000);
		case 6 : return (last_elem * 4.03000000 + 0.97000000);
		case 7 : return (last_elem * 6.60000000 + 0.40000000);
		case 8 : return (last_elem * 20.00000000 + 0.00000000);
		case 9 : return (last_elem * 4.18000000 + 0.82000000);
		case 10 : return (last_elem * 14.00000000 + 1.00000000);

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
	return 11;
}
