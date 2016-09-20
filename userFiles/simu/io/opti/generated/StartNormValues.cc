#include "opti_params.hh"
#include <iostream>
#include <cstdlib>

/*! \brief extact normed results from OptiResults.cc and OptiParams.cc
 * 
 * \param[in] index inde
 * \return normed result
 */
double new_norm_start(int index)
{
	switch (index)
  	{
		case 0 : return 0.00000000;
		case 1 : return 0.00000000;
		case 2 : return 0.00000000;
		case 3 : return 0.04977376;
		case 4 : return 0.12903226;
		case 5 : return 0.00000000;
		case 6 : return 0.12903226;
		case 7 : return 0.12903226;
		case 8 : return 0.00000000;
		case 9 : return 0.00000000;
		case 10 : return 0.00000000;
		case 11 : return 0.00000000;
		case 12 : return 0.00000000;
		case 13 : return 0.00000000;
		case 14 : return 0.00000000;
		case 15 : return 0.00000000;
		case 16 : return 0.35483871;
		case 17 : return 0.00000000;
		case 18 : return 0.11000000;
		case 19 : return 0.09090909;
		case 20 : return 0.21666667;
		case 21 : return 0.25000000;
		case 22 : return 0.12359551;
		case 23 : return 0.12359551;
		case 24 : return 0.16961131;
		case 25 : return 0.08500000;
		case 26 : return 0.73333333;
		case 27 : return 0.73333333;
		case 28 : return 0.91549296;
		case 29 : return 1.00000000;
		case 30 : return 0.07000000;
		case 31 : return 0.10444444;
		case 32 : return 0.05352941;
		case 33 : return 0.40000000;
		case 34 : return 0.40000000;
		case 35 : return 0.40000000;
		case 36 : return 0.94623656;
		case 37 : return 1.00000000;
		case 38 : return 0.01000000;
		case 39 : return 0.06250000;
		case 40 : return 0.25000000;
		case 41 : return 0.20000000;
		case 42 : return 0.20000000;
		case 43 : return 0.20000000;
		case 44 : return 0.40000000;
		case 45 : return 0.40000000;
		case 46 : return 0.40000000;
		case 47 : return 0.46666667;
		case 48 : return 0.20000000;
		case 49 : return 0.50000000;
		case 50 : return 0.40000000;
		case 51 : return 0.03191489;
		case 52 : return 0.07142857;
		case 53 : return 0.80000000;
		case 54 : return 0.50000000;
		case 55 : return 0.50000000;
		case 56 : return 0.29000000;
		case 57 : return 0.50000000;
		case 58 : return 0.50000000;
		case 59 : return 0.33076923;
		case 60 : return 0.50000000;
		case 61 : return 0.10000000;
		case 62 : return 0.50000000;
		case 63 : return 0.50000000;
		case 64 : return 0.50000000;
		case 65 : return 0.00681818;
		case 66 : return 0.50000000;
		case 67 : return 0.00000000;
		case 68 : return 0.50000000;
		case 69 : return 0.00000000;
		case 70 : return 0.50000000;

		default:
			std::cout << "Error: OptiResults is too small: " << index << " !" << std::endl;
			exit(EXIT_FAILURE);
			break;
	}
}

