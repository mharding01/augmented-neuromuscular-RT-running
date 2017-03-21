/*! 
 * \author Nicolas Van der Noot
 * \file eval_main.cc
 * \brief main used to evaluate models
 */

#ifdef EVAL_RUN

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "def_main.hh"
#include "opti_gestion.hh"

#define NB_SPEED_PARAMS_REF 10 ///< number of speed references

// function prototypes
void evaluate_sp_features();

/*! \brief main function
 * 
 * \param[in] argc number of arguments
 * \param[in] argv arguments
 * \return 0 if succeed, error otherwise
 */
int main(int argc, char const *argv[])
{
	evaluate_sp_features();

	return 0;
}

/*! \brief straight walking features evaluated for different speeds
 */
void evaluate_sp_features()
{
	// variable declaration
	int count_test, nb_tests;
	double cur_speed;
	OptiClass *optiClass;

	// vectors
	double speed_ref[NB_SPEED_PARAMS_REF] = {1.25, 1.3, 1.35, 1.4, 1.45, 1.5, 1.55, 1.6, 1.65, 1.7};

	// output file
	std::ofstream out_stream("./out_sp_features.txt");

	if (!out_stream)
	{
		std::cout << "eval_main error: cannot open output stream !" << std::endl;
		exit(EXIT_FAILURE);
	}	

	// get optimization input
	optiClass = get_rnd_OptiClass();


	// number of tests
	nb_tests = 10;

	// loop on all the speeds
	for(int i=0; i<NB_SPEED_PARAMS_REF; i++)
	{
		out_stream << "\n(" << speed_ref[i] << ")" << std::endl;

		cur_speed   = 0.0;

		for(int j=0; j<nb_tests; j++)
		{
			count_test = 0;

			do
			{
				// safety
				if (count_test >= 500)
				{
					std::cout << "problem for speed = " << speed_ref[i] << " m/s" << std::endl;
					break;
				}

				// reset
				optiClass->reset();

				// new parameters
				optiClass->set_v_ref(speed_ref[i]);

				// run simulation
				simu_run(optiClass);

				// outputs
				cur_speed = optiClass->get_v_real();

				std::cout << cur_speed << std::endl;

				count_test++;

			} while (cur_speed < 0.1);

			out_stream << cur_speed << std::endl;
		}
	}
	
	// release RunInOut memory
	delete optiClass;
}

#endif
