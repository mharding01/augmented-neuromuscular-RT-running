/*! 
 * \author Nicolas Van der Noot
 * \file main.cc
 * \brief main project file
 */

#if !defined(OPTI_BIOROB) && !defined(OPTI_MPI)

#include "def_main.hh"
#include "opti_gestion.hh"

/*! \brief main function
 * 
 * \param[in] argc number of arguments
 * \param[in] argv arguments
 * \return 0 if succeed, error otherwise
 */
int main(int argc, char const *argv[])
{
	// variable declaration
	OptiClass *optiClass;

	// get optimization input
	optiClass = get_rnd_OptiClass();

	// run simulation
	simu_run(optiClass);

	// release OptiClass memory
	delete optiClass;

	return 0;
}

#endif
