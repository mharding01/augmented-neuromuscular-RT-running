#include "OptiClass.hh"
#include "opti_params.hh"
#include "cmake_define.h"

/*! \brief constructor
 * 
 * \param[in/out/in,out] name xxx
 */
OptiClass::OptiClass()
{
	reset();
}

/*! \brief destructor
 */
OptiClass::~OptiClass()
{

}

/*! \brief add a new normalized parameter
 * 
 * \param[in] normValue optimized parameter bouded in [0;1] (normalized)
 */
void OptiClass::add_param(double normValue)
{
	optiNorms.push_back(normValue);
	optiParams.push_back(convert_to_optiParams(optiNorms));
}

/*! \brief reset the OptiClass params
 */
void OptiClass::reset()
{
	fitness = 0.0;
	v_ref   = CMAKE_VAR_1;

	v_real = 0.0;

	optiNorms.clear();
	optiParams.clear();
	fitness_details.clear();		
}
