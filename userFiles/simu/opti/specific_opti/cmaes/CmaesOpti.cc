#include "CmaesOpti.hh"
#include <stdlib.h>
#include "opti_params.hh"

/*! \brief constructor
 */
CmaesOpti::CmaesOpti(int N, int nb_gen, int nb_part): EvolOpti(N, nb_gen, nb_part)
{
	// lower and upper bounds
	lb = cmaes_NewDouble(1);
	ub = cmaes_NewDouble(1);

	lb[0] = 0.0;
	ub[0] = 1.0;

	cmaes_boundary_transformation_init(&boundaries, lb, ub, 1);

	// initial values and standard deviation
	xstart = cmaes_NewDouble(N);
	stddev = cmaes_NewDouble(N);

	for(int i=0; i<N; i++)
	{
		xstart[i] = 0.5;
		stddev[i] = 0.25;
		//xstart[i] = new_norm_start(i);
		//stddev[i] = 0.005;
	}

	// CMA-ES main initialization
	fit_val = cmaes_init(&evo, N, xstart, stddev, 0, nb_part, "no");

	if (nb_part != cmaes_Get(&evo, "lambda"))
	{
		std::cout << "Error: lambda (" << cmaes_Get(&evo, "lambda") << ") is different from 'nb_part' (" << nb_part << ") !"  << std::endl;
		exit(EXIT_FAILURE);
	}

	// release memory
	free(xstart);
	free(stddev);
}

/*! \brief destructor
 */
CmaesOpti::~CmaesOpti()
{
	cmaes_exit(&evo);
	cmaes_boundary_transformation_exit(&boundaries);

	free(lb);
	free(ub);
}

/*! \brief initialization
 */
void CmaesOpti::init()
{
	// already done in the constructor
}

/*! \brief finalization
 */
void CmaesOpti::finish()
{
	update_best();
}

/*! \brief fill all the candidates of the current generation
 */
void CmaesOpti::fill_candidates()
{
	// generate lambda new search points, sample population
	pop = cmaes_SamplePopulation(&evo);

	// transform into bounds
	for(int i=0; i<nb_part; i++)
	{
		cmaes_boundary_transformation(&boundaries, pop[i], norm_candidates[i], N);
	}
}

/*! \brief update the distribution
 */
void CmaesOpti::update_distrib()
{
	// fill the fitness
	for(int i=0; i<nb_part; i++)
	{
		fit_val[i] = -fitness_vec[i]; // minus because this library is minimizing
	}

	// update the search distribution used for cmaes_SampleDistribution()
	cmaes_UpdateDistribution(&evo, fit_val);
}

/*! \brief update the best solution
 */
void CmaesOpti::update_best()
{
	// get best estimator for the optimum, xmean
	cmaes_boundary_transformation(&boundaries, (double const *) cmaes_GetPtr(&evo, "xbest")    , x_best_cur , N);
	cmaes_boundary_transformation(&boundaries, (double const *) cmaes_GetPtr(&evo, "xbestever"), x_best_ever, N);

	fit_best_cur  = -cmaes_Get( &evo, "fitness");   // minus because this library is minimizing
	fit_best_ever = -cmaes_Get( &evo, "fbestever"); // minus because this library is minimizing

	update_best_history();
}
