#include "random_ctrl.hh"

#ifdef WIN32
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
