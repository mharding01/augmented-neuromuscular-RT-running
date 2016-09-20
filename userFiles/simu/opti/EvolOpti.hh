/*! 
 * \author Nicolas Van der Noot
 * \file EvolOpti.hh
 * \brief EvolOpti class
 */

#ifndef _EVOL_OPTI_HH_
#define _EVOL_OPTI_HH_

#include <iostream>

/*! \brief evolutionary algorithm
 */
class EvolOpti
{
	public:
		EvolOpti(int N, int nb_gen, int nb_part);
		virtual ~EvolOpti();

		virtual void init() = 0;
		virtual void finish() = 0;

		virtual void fill_candidates() = 0;
		virtual void update_distrib()  = 0;
		virtual void update_best()     = 0;

		void print_results();
		void update_best_history();

		/// get best fitness
		double get_fit_best_ever() const { return fit_best_ever;  };

		/// get best elements
		double get_x_best_ever(int i) const { return x_best_ever[i]; };

		/// get norm_candidates vector i
		double* get_norm_vec(int i) { return norm_candidates[i]; }

		/// get stop signal for the slaves
		double* get_stop_vec() { return stop_vec; }

		/// set the fitness of one particle
		void set_fitness(int i, double value) { fitness_vec[i] = value; }

		/// get history of best fitness
		double get_fit_best_history(int i) { return fit_best_history[i]; }

	protected:
		int N; ///< problem size
		int nb_gen;  ///< number of generations
		int nb_part; ///< number of particles

		double **norm_candidates; ///< whole candidates in the norm bounds [0;1]
		double *stop_vec; ///< vector to stop the slaves

		double *fitness_vec; ///< fitness of all the particles

		double fit_best_cur;  ///< current best fitness
		double fit_best_ever; ///< best fitness ever
		
		double *x_best_cur;  ///< current best solution (range [0;1])
		double *x_best_ever; ///< best solution ever (range [0;1])

		int hist_id; ///< ID of the fitness history
		double *fit_best_history; ///< history of 'fit_best'
};

#endif
