
#ifdef JAVA

#include "visu_past.h"
#include "java_functions.h"
#include "plot_sdl.h"
#include "realtime_functions.h"

#include <math.h>

/*! \brief update the buffers used to store the positions (to go back in visualization)
 * 
 * \param[in,out] realtime real-time structure
 * \param[in] tsim current simulation time [s]
 */
void update_past_visu(Simu_realtime *realtime, double tsim)
{
    // variables declaration
    int i;
    int nb_q;
    int min_tsim_index;
    int buffer_size;

    double *cur_q;
    double *tsim_save;
    double **q_save;

    Realtime_java *java;
    MbsData* mbs_data;

    // variables initialization
    java     = realtime->ext->java;
    mbs_data = realtime->ext->mbs_data;

    nb_q = java->nb_q;

    // no joints in the visualization
    if (!nb_q)
    {
        return;
    }

    min_tsim_index = java->min_tsim_index;

    tsim_save = java->tsim_save;
    q_save    = java->q_save;

    buffer_size = java->buffer_size;

    cur_q = java->cur_q;

    // update the time
    tsim_save[min_tsim_index] = tsim;

    user_realtime_visu(mbs_data, nb_q, cur_q);

    // update the vectors
    for (i=0; i<nb_q; i++)
    {
        q_save[i][min_tsim_index] = cur_q[i];
    }

    // update indexes
    java->max_tsim_index = min_tsim_index;

    min_tsim_index++;

    if (min_tsim_index >= buffer_size)
    {
        min_tsim_index = 0;
        java->flag_buffer_round = 1;
    }

    java->min_tsim_index = min_tsim_index;

    // update min-max
    java->min_tsim = tsim_save[min_tsim_index];
    java->max_tsim = tsim;
}

/*! \brief update the requested q past visualization vector
 * 
 * \param[in,out] realtime real-time structure
 */
void q_past_vector(Simu_realtime *realtime)
{
    // variables declaration
    int i;
    int prev_ind, next_ind;

    double tsim_prev, tsim_next;
    double request_tsim;
    double weight_prev;

    double **q_save;
    double *past_q;
    double *tsim_save;

    Realtime_java *java;

    // variables initialization
    java = realtime->ext->java;

    q_save = java->q_save;
    past_q = java->past_q;

    tsim_save = java->tsim_save;

    request_tsim = java->t_visu_past; ///< requested simulation time [s]

    // get requested indexes: before and after
    guess_tsim_index(request_tsim, tsim_save, java->min_tsim_index, java->max_tsim_index, java->min_tsim, java->max_tsim, java->buffer_size, java->flag_buffer_round, &prev_ind, &next_ind);

    if (prev_ind == next_ind)
    {
        for(i=0; i<java->nb_q; i++)
        {
            past_q[i] = q_save[i][prev_ind];
        }
    }
    else
    {
        // time
        tsim_prev = tsim_save[prev_ind];
        tsim_next = tsim_save[next_ind];

        // weight for the linear interpolation
        if (tsim_next > tsim_prev)
        {
            weight_prev = (tsim_next - request_tsim) / (tsim_next - tsim_prev);
        }
        else
        {
            weight_prev = 0.0;
        }

        for(i=0; i<java->nb_q; i++)
        {
            // linear interpolation
            past_q[i] = weight_prev * q_save[i][prev_ind] + (1 - weight_prev) * q_save[i][next_ind];
        }
    }
}

/*! \brief update the time for past visualization
 * 
 * \param[out] realtime real-time structure
 * \param[int] tsim simulation time [s]
 */
void java_time_visu_past(Simu_realtime *realtime, double tsim)
{
    realtime->ext->java->t_visu_past = tsim;
}

/*! \brief update 't_visu_part' according to the simulation speed factor
 * 
 * \param[in,out] realtime real-time structure
 * \param[in] flag_forward 1 if forward in simulation, 0 fot backward
 */
void update_visu_past(Simu_realtime *realtime, int flag_forward)
{
    double simu_speed_factor;
    double min_tsim, max_tsim;

    Realtime_java *java;

    java = realtime->ext->java;

    min_tsim = java->min_tsim;
    max_tsim = java->max_tsim;

    // simulation speed
    simu_speed_factor = get_simu_speed_factor(realtime->simu_speed_flag);

    // new 't_visu_part'
    if (flag_forward)
    {
        java->t_visu_past += simu_speed_factor * PERIOD_BACK_IN_VISU;
    }   
    else
    {
        java->t_visu_past -= simu_speed_factor * PERIOD_BACK_IN_VISU;
    }

    // bounds
    if (java->t_visu_past < min_tsim)
    {
        java->t_visu_past = min_tsim;
    }

    if (java->t_visu_past > max_tsim)
    {
        java->t_visu_past = max_tsim;
    }

    q_past_vector(realtime);
}

#endif
