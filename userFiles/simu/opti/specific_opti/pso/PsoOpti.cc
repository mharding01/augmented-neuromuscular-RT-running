#include "PsoOpti.hh"
#include "simu_functions.hh"
#include <cmath>

/// limit range
inline double limit(double x, double min, double max) { return (x < min) ? min : (x > max) ? max : x; }

/*! \brief constructor
 */
PsoOpti::PsoOpti(int N, int nb_gen, int nb_part): EvolOpti(N, nb_gen, nb_part)
{
	pop = new Matrix2D(nb_part, N);

	best_part  = new Matrix2D(nb_part, N);
	
	velocity = new Matrix2D(nb_part, N);

	best_swarm = new double[N];
	best_swarm_cur = new double[N];

	for(int i=0; i<N; i++)
	{
		best_swarm[i] = 0.0;
		best_swarm_cur[i] = 0.0;
	}

	best_fit_part = new double[nb_part];

	for(int i=0; i<nb_part; i++)
	{
		best_fit_part[i] = 0.0;
	}

	best_fit_swarm = 0.0;
	best_fit_swarm_cur = 0.0;

	// set all values to 0
	pop->reset();
	best_part->reset();
	velocity->reset();

	lb = 0.0;
	ub = 1.0;

	diff_b = ub - lb;

	// safety
	if (diff_b <= 0.0)
	{
		std::cout << "Error in PSO: upper bound not above lower bound !" << std::endl;
		exit(EXIT_FAILURE);
	}

	flag_first = 0;

	// factors
	omega  = 0.9; // last velocity
	phi_sw = 0.6; // for 'best_swarm'
	phi_pt = 0.6; // for 'best_part'
}

/*! \brief destructor
 */
PsoOpti::~PsoOpti()
{
	delete[] best_fit_part;
	delete[] best_swarm;
	delete[] best_swarm_cur;

	delete velocity;

	delete best_part;

	delete pop;
}

/*! \brief initialization
 */
void PsoOpti::init()
{
	// already done in the constructor
}

/*! \brief finalization
 */
void PsoOpti::finish()
{
	update_best();
}

/*! \brief fill all the candidates of the current generation
 */
void PsoOpti::fill_candidates()
{
	double new_vel, new_pos;
	double rnd_sw, rnd_pt;

	if (!flag_first) // first iteration
	{
		for(int i=0; i<nb_part; i++)
		{
			for(int j=0; j<N; j++)
			{
				pop->set(i, j, lb + diff_b * rnd_simu());
			}
		}
	}
	else // next iterations
	{
		for(int i=0; i<nb_part; i++)
		{
			for(int j=0; j<N; j++)
			{
				// update velocity
				rnd_sw = rnd_simu();
				rnd_pt = rnd_simu();

				new_vel = omega  * velocity->get(i,j)
						+ phi_sw * rnd_sw * (best_swarm[j] - pop->get(i,j))
						+ phi_pt * rnd_pt * (best_part->get(i,j) - pop->get(i,j));

				velocity->set(i, j, new_vel);

				// update position
				new_pos = pop->get(i,j) + velocity->get(i,j);

				new_pos = limit(new_pos, lb, ub); // limit position in range [lb;ub]

				pop->set(i, j, new_pos);
			}
		}
	}

	// set pop to generic 'EvolOpti' variable
	for(int i=0; i<nb_part; i++)
	{
		for(int j=0; j<N; j++)
		{
			norm_candidates[i][j] = pop->get(i,j);
		}
	}
}

/*! \brief update the distribution
 */
void PsoOpti::update_distrib()
{
	double cur_fit;

	if (!flag_first) // first iteration
	{
		// assume first particle has best fitness (corrected later)
		best_fit_swarm   = fitness_vec[0];
		best_fit_part[0] = fitness_vec[0];

		for(int i=0; i<N; i++)
		{
			best_swarm[i] = pop->get(0,i);

			best_part->set(0, i, pop->get(0,i));	
		}

		// loop on remaining particles
		for(int i=1; i<nb_part; i++)
		{
			cur_fit = fitness_vec[i];

			best_fit_part[i] = cur_fit;

			for(int j=0; j<N; j++)
			{
				best_part->set(i, j, pop->get(i,j));
			}

			if (cur_fit > best_fit_swarm)
			{
				best_fit_swarm = cur_fit;

				for(int j=0; j<N; j++)
				{
					best_swarm[j] = pop->get(i,j);
				}
			}
		}

		flag_first = 1;
	}
	else // next iterations
	{
		for(int i=0; i<nb_part; i++)
		{
			cur_fit = fitness_vec[i];

			// update best particle history
			if (cur_fit > best_fit_part[i])
			{
				best_fit_part[i] = cur_fit;

				for(int j=0; j<N; j++)
				{
					best_part->set(i, j, pop->get(i,j));
				}
			}

			// update best swarm history
			if (cur_fit > best_fit_swarm)
			{
				best_fit_swarm = cur_fit;

				for(int j=0; j<N; j++)
				{
					best_swarm[j] = pop->get(i,j);
				}
			}
		}
	}

	// current generation best
	best_fit_swarm_cur = fitness_vec[0];

	for(int i=0; i<N; i++)
	{
		best_swarm_cur[i] = pop->get(0,i);
	}

	for(int i=1; i<nb_part; i++)
	{
		cur_fit = fitness_vec[i];

		// update current best swarm
		if (cur_fit > best_fit_swarm_cur)
		{
			best_fit_swarm_cur = cur_fit;

			for(int j=0; j<N; j++)
			{
				best_swarm_cur[j] = pop->get(i,j);
			}
		}
	}
}

/*! \brief update the best solution
 */
void PsoOpti::update_best()
{
	fit_best_ever = best_fit_swarm;
	fit_best_cur  = best_fit_swarm_cur;

	for(int i=0; i<N; i++)
	{
		x_best_ever[i] = best_swarm[i];
		x_best_cur[i]  = best_swarm_cur[i];
	}

	update_best_history();
}
