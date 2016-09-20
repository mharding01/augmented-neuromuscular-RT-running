#include "opti_gestion.hh"
#include "opti_params.hh"
#include "user_IO.h"
#include "simu_functions.hh"
#include "random_ctrl.hh"

/*! \brief generate an OptiClass instance with randon numbers
 * 
 * \return randomly initialized instance of OptiClass
 */
OptiClass* get_rnd_OptiClass()
{
	// variables declaration
	OptiClass *optiClass;
	struct timeval timeValue;

	// seed for random
	gettimeofday(&timeValue, NULL);
	srand(timeValue.tv_usec * timeValue.tv_sec);

	// new OptiClass
	optiClass = new OptiClass();

	// add random parameters
	for(int i=0; i<get_nb_optiParams(); i++)
	{
		optiClass->add_param(rnd_simu());
	}

	return optiClass;
}

/*! \brief return the fitness value extracted from the OptiClass
 * 
 * \param[in] mbs_data Robotran structure
 * \return fitness value
 */
double extract_OptiClass(MbsData *mbs_data)
{
	return static_cast<OptiClass*>(mbs_data->user_IO->optiClass)->get_fitness();
}

/*! \brief return the fitness stages values extracted from the OptiClass
 * 
 * \param[in] mbs_data Robotran structure
 * \return fitness value
 */
std::vector<double> extract_OptiClass_details(MbsData *mbs_data)
{
	return static_cast<OptiClass*>(mbs_data->user_IO->optiClass)->get_fitness_details();
}
