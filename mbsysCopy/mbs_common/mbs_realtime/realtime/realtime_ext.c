
#ifdef REAL_TIME

#include "realtime_ext.h"
#include "realtime.h"
#include "time_functions.h"

#include <stdlib.h>

#ifdef SDL
#include "plot_sdl.h"
#include "events_sdl.h"
#endif

#ifdef JAVA
#include "java_functions.h"
#endif

#ifdef SDL

/*! \brief initialize SDL real-time variables
 * 
 * \param[in] screen_sdl SDL main data structure
 * \return real-time structure for SDL
 */
Realtime_sdl* init_realtime_sdl(void *screen_sdl)
{
    // variable (pointer) to return
    Realtime_sdl *sdl;

    // memory allocation
    sdl = (Realtime_sdl*) malloc(sizeof(Realtime_sdl));

    sdl->mouse_init_x                     = 0;
    sdl->mouse_init_y                     = 0;
    sdl->mouse_delta_x                    = 0;
    sdl->mouse_delta_y                    = 0;
    sdl->mouse_cur_x                      = 0;
    sdl->mouse_cur_y                      = 0;
    sdl->mouse_wheel_flag                 = 0;
    sdl->start_mouse_usec                 = 0;
    sdl->last_action_break_usec           = 0;
    sdl->next_user_keyboard_event_usec    = 0;
    sdl->next_generic_keyboard_event_usec = 0;
    sdl->last_mouse_event_usec            = 0;
    sdl->mouse_left_pressed               = 0;
    sdl->mouse_right_pressed              = 0;

    sdl->screen_sdl = screen_sdl;

    return sdl;
}

/*! \brief free SDL real-time structure
 * 
 * \param[out] sdl real-time structure for SDL
 */
void free_realtime_sdl(Realtime_sdl *sdl)
{
    free_screen_sdl(sdl->screen_sdl);
    free(sdl);
}
#endif

#ifdef JAVA

/*! \brief initialize Java real-time variables
 * 
 * \param[in] realtime_options options for real-time
 * \return real-time structure for java visualization
 */
Realtime_java* init_realtime_java(void *realtime_options, MbsData* mbs_data)
{
    int i, j;
    int nb_q, buffer_size;

    double t0;

    Realtime_option *options;
	Realtime_java *java;

    // variable (pointer) to return
    options = (Realtime_option*) realtime_options;


    buffer_size = options->buffer_size;
    nb_q = options->nb_q;
    

    // memory allocation
    java = (Realtime_java*) malloc(sizeof(Realtime_java));

    t0 = options->t0;

    java->buffer_size = buffer_size;

    java->nb_q = nb_q;

    // view point of the simulation
    java->change_viewpoint = 1;
    java->cur_viewpoint    = options->start_viewpoint-1;

    java->visu_past_flag = 0;
    java->t_visu_past    = 0.0;

    java->mbs_file = options->mbs_file;

    java->cur_tsim = t0;

    java->min_tsim_index = 0;
    java->max_tsim_index = buffer_size-1;

    java->min_tsim = t0;
    java->max_tsim = t0 + SAFETY_TIME;

    java->flag_buffer_round = 0;

    java->last_past_q_flag = 0;

    java->cur_q = (double*) malloc(nb_q * sizeof(double));
    for(i=0; i<nb_q; i++)
    {
        java->cur_q[i] = 0.0;
    }

    java->past_q = (double*) malloc(nb_q * sizeof(double));
    for(i=0; i<nb_q; i++)
    {
        java->past_q[i] = 0.0;
    }

    java->anim_q = (double*) malloc(nb_q * sizeof(double));
    for(i=0; i<nb_q; i++)
    {
        java->anim_q[i] = 0.0;
    }

    // full time save
    java->tsim_save = (double*) malloc(buffer_size * sizeof(double));
    for(i=0; i<buffer_size; i++)
    {
        java->tsim_save[i] = t0;
    }

    // full q (positions) save
    java->q_save = (double**) malloc(nb_q * sizeof(double*));
    for (i=0; i<nb_q; i++)
    {
        java->q_save[i] = (double*) malloc(buffer_size * sizeof(double));

        for(j=0; j<buffer_size; j++)
        {
            java->q_save[i][j] = 0.0; 
        }
    }

    user_realtime_visu(mbs_data, nb_q, java->cur_q);

    init_jni(java, nb_q, java->cur_q, options->mbs_file, options->start_viewpoint);

    return java;
}

/*! \brief free Java real-time structure
 * 
 * \param[out] java real-time structure for java visualization
 */
void free_realtime_java(Realtime_java *java)
{
    int i;

    free(java->tsim_save);
    free(java->cur_q);
    free(java->past_q);
    free(java->anim_q);

    for(i=0; i<java->nb_q; i++)
    {
        free(java->q_save[i]);
    }
    free(java->q_save);

    free_jni(java->jni_struct);

    free(java);
}
#endif

#ifdef SDL

/*! \brief hanfles the events coming from the user
 * 
 * \param[in,out] realtime real-time structure
 * \param[in] tsim simulation time [s]
 */
void events_simu(Simu_realtime *realtime, double tsim)
{
    int cur_t_usec;

    cur_t_usec = t_usec(realtime->init_t_sec, realtime->init_t_usec);

    // -- handle events -- //
    events_sdl(realtime, cur_t_usec);

    // break in the simulation if needed
    if (realtime->simu_break == 1)
    {
        break_gestion(realtime, tsim);
    }
}

/*! \brief hanfles break
 * 
 * \param[in,out] realtime real-time structure
 * \param[in] tsim simulation time [s]
 */
void last_break_gestion(Simu_realtime *realtime, double tsim)
{
    if (realtime->last_break)
    {
        realtime->simu_break = 1;

        events_simu(realtime, tsim);
    }  
}

#endif

#endif
