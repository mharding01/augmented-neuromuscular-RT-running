#include "EvolOpti.hh"
#include <stdlib.h>
#include <stdio.h>

/*! \brief constructor
 */
EvolOpti::EvolOpti(int N, int nb_gen, int nb_part)
{
	this->N = N;
	this->nb_gen  = nb_gen;
	this->nb_part = nb_part;

	// candidates
	norm_candidates = new double*[nb_part];

	for(int i=0; i<nb_part; i++)
	{
		norm_candidates[i] = new double[N];

		for(int j=0; j<N; j++)
		{
			norm_candidates[i][j] = 0.0;
		}
	}

	// stop vector
	stop_vec = new double[N];

	for(int i=0; i<N; i++)
	{
		stop_vec[i] = -1.0;
	}

	// best solution
	fit_best_cur  = 0.0;
	fit_best_ever = 0.0;

	x_best_cur  = new double[N];
	x_best_ever = new double[N];

	for(int i=0; i<N; i++)
	{
		x_best_cur[i]  = 0.0;
		x_best_ever[i] = 0.0;
	}

	// fitness
	fitness_vec = new double[nb_part];

	for(int i=0; i<nb_part; i++)
	{
		fitness_vec[i] = 0.0;
	}

	// history fitness
	hist_id = 0;

	fit_best_history = new double[nb_gen];

	for(int i=0; i<nb_gen; i++)
	{
		fit_best_history[i] = 0.0;
	}
}

/*! \brief destructor
 */
EvolOpti::~EvolOpti()
{
	// history fitness
	delete[] fit_best_history;

	// fitness
	delete[] fitness_vec;

	// best solution
	delete[] x_best_cur;
	delete[] x_best_ever;

	// stop vector
	delete[] stop_vec;

	// candidates
	for(int i=0; i<nb_part; i++)
	{
		delete[] norm_candidates[i];
	}
	delete[] norm_candidates;
}

/*! \brief print the optimization results
 */
void EvolOpti::print_results()
{
	int count;

	// best fitness
	printf("fitness: %f\n\n", fit_best_ever);

	// fitness history
	count = 0;

	printf("fitness history:\n");

	for(int i=0; i<nb_gen; i++)
	{
		count++;

		if (count == 1)
		{
			printf("  %f", fit_best_history[i]);
		}
		else if(count >= 10)
		{
			printf(" ; %f\n", fit_best_history[i]);
			count = 0;
		}
		else
		{
			printf(" ; %f", fit_best_history[i]);
		}
	}
	printf("\n");

	// best values
	count = 0;

	printf("values:\n");

	for(int i=0; i<N; i++)
	{
		count++;

		if (count == 1)
		{
			printf("  %f", x_best_ever[i]);
		}
		else if(count >= 10)
		{
			printf(" ; %f\n", x_best_ever[i]);
			count = 0;
		}
		else
		{
			printf(" ; %f", x_best_ever[i]);
		}
	}
	printf("\n");
}

/*! \brief update the best solution
 */
void EvolOpti::update_best_history()
{
	// safety
	if (hist_id >= nb_gen)
	{
		return;
	}
	
	fit_best_history[hist_id] = fit_best_cur;

	hist_id++;
}
