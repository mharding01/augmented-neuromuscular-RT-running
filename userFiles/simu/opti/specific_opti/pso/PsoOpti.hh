/*! 
 * \author Nicolas Van der Noot
 * \file PsoOpti.hh
 * \brief PsoOpti class
 */

#ifndef _PSO_OPTI_HH_
#define _PSO_OPTI_HH_

#include "EvolOpti.hh"
#include "Matrix2D.hh"

/*! \brief PSO optimization algorithm
 */
class PsoOpti: public EvolOpti
{
	public:
		PsoOpti(int N, int nb_gen, int nb_part);
		virtual ~PsoOpti();

		virtual void init();
		virtual void finish();

		virtual void fill_candidates();
		virtual void update_distrib();
		virtual void update_best();

	private:
		Matrix2D *pop; ///< population position

		double   *best_swarm; ///< best position of the whole swarm
		double   *best_swarm_cur; ///< best position of the whole swarm for the current generation
		Matrix2D *best_part;  ///< best position of the different particles

		Matrix2D *velocity; ///< particles velocity

		double *best_fit_part; ///< best fitness of each particle
		double best_fit_swarm; ///< best fitness of the whole swarm
		double best_fit_swarm_cur; ///< best fitness of the whole swarm for the current generation

		double lb; ///< lower bounds for the optimized parameters
		double ub; ///< upper bounds for the optimized parameters
		double diff_b; ///< difference between bounds

		int flag_first; ///< 1 if first iteration done, 0 otherwise

		double omega;  ///< last velocity factor
		double phi_sw; ///< factor related to 'best_swarm'
		double phi_pt; ///< factor related to 'best_part'
};

#endif
