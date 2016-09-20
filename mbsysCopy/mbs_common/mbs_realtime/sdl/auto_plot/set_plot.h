/*! 
 * \author Nicolas Van der Noot
 * \file set_plot.h
 * \brief set_plot function called by the user to plot a curve
 */

#ifndef _SET_PLOT_H_
#define _SET_PLOT_H_

#include "user_realtime.h"

#ifdef SDL
    #include "plot_sdl.h"
    void init_set_plot(Screen_sdl *screen_sdl);
    void free_set_plot();
#endif

#endif
