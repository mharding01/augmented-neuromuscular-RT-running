/*! 
 * \author Nicolas Van der Noot
 * \file mpi_main.cc
 * \brief main file for the MPI optimizations
 */

#ifdef OPTI_MPI

// MPI includes
#include <mpi.h>

// personal includes
#include "OptiClass.hh"
#include "opti_params.hh"
#include "def_main.hh"
#include "random_ctrl.hh"
#include "EvolOpti.hh"
#include "CmaesOpti.hh"
#include "PsoOpti.hh"

#define MASTER 0 // rank ID of the master (do not modify)

#define NB_PART 50 	     	// number of particles
#define NB_GENERATION 1000   // number of generations

#define TAG_END NB_PART+1

/*
 * Uncomment the following line to test the optimization with a simple example.
 */
//#define TEST_OPTI

/// square function
inline double square(double x) { return x*x; }

/*! \brief test of a simple fitness function (to maximize)
 * 
 * \param[in] x parameters vector in [0 ; 1]
 * \param[in] size size of 'x'
 * \return fitness value
 */
double fitness_test(double *x, int size)
{
	double sum = 0.0;

	for(int i=0; i<size; i++)
	{
		sum -= square(x[i] - 0.5);
	}

	return sum;
}

/*! \brief fitness function (to maximize)
 * 
 * \param[in] x parameters vector in [0 ; 1]
 * \param[in] size size of 'x'
 * \param[in] optiClass optimization class for the robot
 * \return fitness value
 */
double fitness_function(double *x, int size, OptiClass *optiClass)
{
	#ifdef TEST_OPTI // test with simple optimization

		return fitness_test(x, size);

	#else // robot optimization

		optiClass->reset();

		for(int i=0; i<size; i++)
		{
			optiClass->add_param(x[i]);
		}

		// run simulation(s) to compute the fitness
		return mean_fitness(optiClass, 2);
		//return = speed_range_fitness(optiClass);

	#endif
}

/*! \brief master, in charge of the optimization gestion
 * 
 * \param[in] np number of processes
 * \param[in] N size of the problem (size of the optimized vector)
 * \param[in] nb_gen number of generations
 */
void master_process(int np, int N, int nb_gen)
{
	// variables declaration
	int nb_slave;      // number of slaves
	double fitness;    // fitness function
	int m, who, tag;

	MPI_Status status;    // communication status
	EvolOpti *opti;       // evolutionary optimization
	OptiClass *optiClass; // optimization class for the robot

	FILE* fid; // file to print results

	// -- initialization -- //

	nb_slave = np - 1; // number of processes - the master

	// safety
	if (nb_slave < 0)
	{
		std::cout << "Error: number of process is negative !" << std::endl;
		exit(EXIT_FAILURE);
	}
	else if (nb_slave == 1)
	{
		std::cout << "Error: number of process cannot be 1 (i.e. do not call MPI with two cores) !" << std::endl;
		exit(EXIT_FAILURE);
	}

	if (nb_slave > NB_PART)
	{
		std::cout << "Error: number of slaves process is bigger than number of particles !" << std::endl;
		exit(EXIT_FAILURE);
	}

	fitness = 0.0;

	// optimization class creation (you can change the optimization algortihm here)
	opti = new CmaesOpti(N, nb_gen, NB_PART);

	// optimization init
	opti->init();

	// in case nb_slave == 0 (only master is working)
	if(!nb_slave)
	{
		optiClass = new OptiClass();
	}


	// -- loop -- //

	for(int i=0; i<nb_gen; i++)
	{
		// get candidates
		opti->fill_candidates();

		// in case nb_slave == 0 (only master is working)
		if(!nb_slave)
		{
			for(int j=0; j<NB_PART; j++)
			{
				fitness = fitness_function(opti->get_norm_vec(j), N, optiClass);
				opti->set_fitness(j, fitness);
			}
		}
		else
		{
			// send candidates to all slaves 
			for(m=0; m<nb_slave; m++)
			{
				MPI_Send(opti->get_norm_vec(m), N, MPI_DOUBLE, m+1, m, MPI_COMM_WORLD);
			}

			// send candidates to free slaves and get fitness from them
			while(m<NB_PART)
			{
				MPI_Recv(&fitness, 1, MPI_DOUBLE, MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
				who = status.MPI_SOURCE; // id slave
				tag = status.MPI_TAG; // id particle
				opti->set_fitness(tag, fitness);
				MPI_Send(opti->get_norm_vec(m), N, MPI_DOUBLE, who, m, MPI_COMM_WORLD);
				m++;
			}

			// get fitnesses from last slaves
			for(m=0; m<nb_slave; m++)
			{
				MPI_Recv(&fitness, 1, MPI_DOUBLE, MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
				who = status.MPI_SOURCE; //id slave
				tag = status.MPI_TAG; //id particle
				opti->set_fitness(tag, fitness);

				// stop slaves
				if (i == (nb_gen-1))
				{
					MPI_Send(opti->get_norm_vec(m), N, MPI_DOUBLE, who, TAG_END, MPI_COMM_WORLD);
				}
			}
		}

		// update the population distribution
		opti->update_distrib();

		// update the best fitness
		opti->update_best();

		// print current best result
		fid = fopen("./opti_res.txt", "w");

		if(fid == NULL)
		{
			std::cout << "Error: cannot open 'opti_res.txt' !" << std::endl;
			break;
		}

		fprintf(fid, "generation: %d/%d\n\n", i+1, nb_gen);
		
		fprintf(fid, "best fitness: %g\n\n", opti->get_fit_best_ever());

		fprintf(fid, "best particle:\n");
		for(int j=0; j<N; j++)
		{
			fprintf(fid, "%g\n", opti->get_x_best_ever(j));
		}

		fclose(fid);

		std::cout << "iteration " << i+1 << " performed, best fitness : " << opti->get_fit_best_history(i) << "" << std::endl;
	}


	// -- finalization -- //

	// in case nb_slave == 0 (only master is working)
	if(!nb_slave)
	{
		delete optiClass;
	}

	// finilization
	opti->finish();

	// print results
	opti->print_results();

	// release memory
	delete opti;

	printf("close master\n");
}

/*! \brief slave, in charge of computing the fitness function
 *
 * \param[in] myrank rank of this slave
 * \param[in] N size of the problem (size of the optimized vector)
 */
void slave_process(int myrank, int N)
{
	// variables declaration
	double fitness;    // fitness function
	double* candidate; // current candidate
	MPI_Status status; // communication status
	OptiClass *optiClass; // optimization class for the robot
	int tag;

	// -- initialization -- //

	fitness = 0.0;

	candidate = new double[N];

	optiClass = new OptiClass();

	// -- loop -- //

	// receive candidate from the master
	MPI_Recv(candidate, N, MPI_DOUBLE, MASTER, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
	tag = status.MPI_TAG; //id particles

	while(tag != TAG_END)
	{
		// compute fitness
		fitness = fitness_function(candidate, N, optiClass);

		// send fitness to the master
		MPI_Send(&fitness, 1, MPI_DOUBLE, MASTER, tag, MPI_COMM_WORLD);

		// receive candidate from the master
		MPI_Recv(candidate, N, MPI_DOUBLE, MASTER, MPI_ANY_TAG, MPI_COMM_WORLD, &status);
		tag = status.MPI_TAG;
	}

	// -- finalization -- //

	delete optiClass;

	delete[] candidate;

	printf("close slave %d\n", myrank);
}

/*! \brief main function
 * 
 * \param[in] argc number of arguments
 * \param[in] argv arguments
 * \return 0 if succeed, error otherwise
 */
int main(int argc, char *argv[])
{
	// variables declaration
	int N; // number of parameters to optimize
	int myrank, np ; //process rank and number of processors
	struct timeval timeValue; // for seed

	// seed for random
	gettimeofday(&timeValue, NULL);
	srand(timeValue.tv_usec * timeValue.tv_sec);

	// initialize MPI
	MPI_Init(&argc, &argv);

	// size of the associated group
	MPI_Comm_size(MPI_COMM_WORLD, &np);

	// rank of the calling process in the communicator
	MPI_Comm_rank(MPI_COMM_WORLD, &myrank);

	#ifdef TEST_OPTI // test with simple optimization
		N = 50;
	#else // robot optimization
		N = get_nb_optiParams();
	#endif

	if(myrank == MASTER) // master
	{
		master_process(np, N, NB_GENERATION);
	}
	else // slave
	{
		slave_process(myrank, N);
	}

	// close MPI
	MPI_Finalize();

	return 0;
}

#endif
