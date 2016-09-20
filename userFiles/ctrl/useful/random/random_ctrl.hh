/*! 
 * \author Nicolas Van der Noot
 * \file random_ctrl.hh
 * \brief random tools like seed
 */

#ifndef _RANDOM_CTRL_HH_
#define _RANDOM_CTRL_HH_

#ifdef UNIX
	#include <sys/time.h>
#else
	#include <time.h>
	#include <sys/timeb.h>
#endif

#ifdef WIN32
// structure timeval
typedef struct timeval {
	long tv_sec;
	long tv_usec;
} timeval;

// function prototype
int gettimeofday (struct timeval *tp, void *tz);
#endif

#endif
