/*===========================================================================*
	*
	*  user_sf_IO.c
	*    
	*  Project:    PendulumSpringC
	* 
	*  Generation date: 14-Nov-2014 18:28:15
	* 
	*  (c) Universite catholique de Louvain
	*      2, Place du Levant 
	*      1348 Louvain-la-Neuve 
	*  http://www.robotran.be// 
	*  
 /*===========================================================================*/

#include <stdlib.h>
#include "user_IO.h"

#ifdef UNIX
	#include <sys/time.h>
#else
	#include <time.h>
	#include <sys/timeb.h>
#endif

#if defined(WIN32) && defined(REAL_TIME)

#include "time_functions.h"

#elif defined(WIN32) && !defined(REAL_TIME)
// structure timeval
typedef struct timeval {
	long tv_sec;
	long tv_usec;
} timeval;

/*! \brief get current time fot Windows
 * 
 * \param[in] tz NULL
 * \param[out] tp timeval structure to fill
 * \return 0
 */
int gettimeofday (struct timeval *tp, void *tz)
{
    struct _timeb timebuffer;
    _ftime (&timebuffer);
    tp->tv_sec  = timebuffer.time;
    tp->tv_usec = timebuffer.millitm * 1000;
    return 0;
}
#endif

/*! \brief initialize the UserIO struct
 * 
 * \return initialized structure
 */
UserIO* mbs_new_user_IO()
{
	// variables declaration
	UserIO *uvs;
	struct timeval timeValue;

	// initialize structure
	uvs = (UserIO*) malloc(sizeof(UserIO));

	// user options
	uvs->options = init_SimuOptions();

	// absolute initial time of the simulation
	gettimeofday( &timeValue, NULL );

	uvs->init_t_sec  = (int)timeValue.tv_sec;
	uvs->init_t_usec = (int)timeValue.tv_usec;

	return uvs;
}

/*! \brief release the UserIO struct memory
 * 
 * \param[out] uvs UserIO struct to release
 */
void mbs_delete_user_IO(UserIO *uvs)
{
	free_SimuOptions(uvs->options);

	free(uvs);
}
