/*! 
 * \author Nicolas Van der Noot and Jesse van den Kieboom
 * \file biorob_main.cc
 * \brief main file for the BioRob optimizations
 */

#ifdef OPTI_BIOROB

// generic includes
#include <optimization/messages.hh>
#include <optimization/dispatcher.hh>
#include <signal.h>
#include <stdio.h>

// personal includes
#include "OptiClass.hh"
#include "opti_params.hh"
#include "def_main.hh"

using namespace std;
using namespace optimization::messages;

// Dispatcher subclass definition
class Dispatcher : public optimization::Dispatcher
{
	protected:
		virtual bool UseMainLoop() const;
		virtual bool RunTask();
};

// global dispatcher object
Dispatcher dispatcher_opti;

bool
Dispatcher::UseMainLoop() const
{
	return false;
}

bool
Dispatcher::RunTask()
{
	// variable declaration
	double cur_param, fitness;
	char str[10];

	OptiClass *optiClass;

	// cluster tasks
	task::Response response;
	response.set_status(task::Response::Success);
	response.set_id(0);

	task::Response::Fitness *maxfitness = response.add_fitness();
	maxfitness->set_name("f");

	//-------------------------------------------------------------------------//

	// get optimization input
	optiClass = new OptiClass();

	for(int i=0; i<get_nb_optiParams(); i++)
	{
		sprintf(str,"x%d", i);
		Parameter(str, cur_param);

		optiClass->add_param(cur_param);
	}

	// run simulation(s) to compute the fitness
	fitness = mean_fitness(optiClass, 3);
	//fitness = speed_range_fitness(optiClass);

	// release OptiClass memory
	delete optiClass;
	
	// finally, set fitness value
	maxfitness->set_value(fitness);

	//-------------------------------------------------------------------------//

	WriteResponse(response);

	return true;
}

static void
nicely_stop(int sig)
{
	dispatcher_opti.Stop();
}

/*! \brief main function
 * 
 * \param[in] argc number of arguments
 * \param[in] argv arguments
 * \return 0 if succeed, error otherwise
 */
int main (int argc, char const* argv[])
{
	signal(SIGTERM, nicely_stop);

	if (dispatcher_opti.Run())
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

#endif
