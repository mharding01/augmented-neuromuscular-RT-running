/*! 
 * \author Nicolas Van der Noot
 * \file CmaesOpti.hh
 * \brief CmaesOpti class
 */

#ifndef _CMAES_OPTI_HH_
#define _CMAES_OPTI_HH_

#include "EvolOpti.hh"

#ifdef __cplusplus
extern "C" {
#endif
	#include "cmaes.h"
	#include "cmaes_interface.h"
	#include "boundary_transformation.h"
#ifdef __cplusplus
}
#endif

/*! \brief CMA-ES optimization algorithm
 */
class CmaesOpti: public EvolOpti
{
	public:
		CmaesOpti(int N, int nb_gen, int nb_part);
		virtual ~CmaesOpti();

		virtual void init();
		virtual void finish();

		virtual void fill_candidates();
		virtual void update_distrib();
		virtual void update_best();

	private:
		cmaes_t evo; ///< CMA-ES type struct or "object"

		double *lb; ///< lower bounds for the optimized parameters
		double *ub; ///< upper bounds for the optimized parameters
		cmaes_boundary_transformation_t boundaries; ///< boundaries for the optimization

		double *fit_val; ///< fitness values

		double *xstart; ///< initial values
		double *stddev; ///< initial standard deviations

		double *const*pop; ///< whole population (cma-es algorithm special bounds)
};

#endif
