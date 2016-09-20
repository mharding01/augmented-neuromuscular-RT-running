
#ifndef _USER_REALTIME_H_
#define _USER_REALTIME_H_

#ifdef REAL_TIME

#include "mbs_data.h"
#include "realtime.h"
#include "user_realtime_visu.h"
#include <stdint.h>

#ifdef SDL
#include "SDL.h"

#ifdef __cplusplus
extern "C" {
#endif
    void user_realtime_plot(MbsData* mbs_data);
    void user_keyboard(MbsData* mbs_data, Simu_realtime *realtime, int cur_t_usec, const Uint8 *keystates);
    void user_joystick_axes(MbsData* mbs_data, Simu_realtime *realtime, int nb_joysticks);
    void user_joystick_buttons(MbsData* mbs_data, int buttonID);
    void wait_key(Simu_realtime *realtime, int cur_t_usec, double tsim);
#ifdef __cplusplus
}
#endif
#endif

#endif

#ifdef __cplusplus
extern "C" {
    void set_plot(double value, const char* label);
    }
#else
    void set_plot(double value, char* label);
#endif

#endif
