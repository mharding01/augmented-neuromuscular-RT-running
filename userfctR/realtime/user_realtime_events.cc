/*! 
 * \author Nicolas Van der Noot
 * \file user_realtime_events.c
 * \brief Handles the events coming from the keyboard or from other sources via the SDL framework
 *
 * In order to use C++ features, you just need to change the extension of this file (.c) to .cc (or .cpp).
 */
 
#ifdef SDL

#include "realtime.h"
#include "events_sdl.h"
#include "user_realtime.h"

#include "user_model.h"
#include "UserCtrl.hh"
#include "CppInterface.hh"

// velocity request for CoMan
#define MIN_VELOCITY_REQUEST 0.4
#define MAX_VELOCITY_REQUEST 0.9
#define MEAN_VELOCITY_REQUEST (0.5 * (MIN_VELOCITY_REQUEST + MAX_VELOCITY_REQUEST))
#define DIFF_VELOCITY_REQUEST (MAX_VELOCITY_REQUEST - MIN_VELOCITY_REQUEST)
#define SEMI_DIFF_VELOCITY_REQUEST (DIFF_VELOCITY_REQUEST / 2.0)

#define MAX_KEYBOARD_COMMAND 100.0

inline int limit(int x, int min, int max){ return (x < min) ? min : (x > max) ? max : x; }

/*! \brief handle inputs comming from the keyboard
 * 
 * \param[in,out] mbs_data Robotran main structure
 * \param[in] realtime real-time structure
 * \param[in] cur_t_usec curent time [us]
 * \param[in] keystates state of the keys (from the keyboard)
 *
 * Use keystates['key code'] (see https://wiki.libsdl.org/SDL_Scancode)
 * to detect when the corresponding key is pressed and add your own functions
 * according to the key pressed.
 *
 * Call 'wait_key(realtime, cur_t_usec, time);' in the statement where you replace 'time'
 * by a time in seconds if you want to wait before detecting a new user command.
 * Pay attention, when you call 'wait_key', the program is automatically kept awake
 * for a few seconds. Consequently, the process can not be realeased after a few seconds
 * during a break in case 'wait_key' is always called.
 *
 * example:
 *      if (keystates[SDL_SCANCODE_UP]) 
 *      {
 *          mbs_data->user_IO->my_command++;
 *          wait_key(realtime, cur_t_usec, 0.1);
 *      }
 *      else if (keystates[SDL_SCANCODE_DOWN]) 
 *      {
 *          mbs_data->user_IO->my_command--;
 *          wait_key(realtime, cur_t_usec, 0.1);
 *      }
 */
void user_keyboard(MbsData* mbs_data, Simu_realtime *realtime, int cur_t_usec, const Uint8 *keystates)
{
	int v_input;
	int key_a, key_b;

	UserCtrl *uCtrl;
	CppInterface *cppInterface;

	cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);
	uCtrl = cppInterface->get_ctrl()->get_user_ctrl();

	v_input = uCtrl->get_v_input();

	key_a = uCtrl->get_keyboard_input(0, 0);
	key_b = uCtrl->get_keyboard_input(0, 1);

	// Up arrow: increase speed
	if (keystates[SDL_SCANCODE_UP]) 
	{
		v_input++;
		key_a++;
		wait_key(realtime, cur_t_usec, 0.01);
	}

	// Down arrow: decrease speed
	else if (keystates[SDL_SCANCODE_DOWN]) 
	{
		v_input--;
		key_a--;
		wait_key(realtime, cur_t_usec, 0.01);
	}

	// Left arrow
	else if (keystates[SDL_SCANCODE_LEFT]) 
	{
		v_input--;
		key_b++;
		wait_key(realtime, cur_t_usec, 0.01);
	}

	// Right arrow
	else if (keystates[SDL_SCANCODE_RIGHT]) 
	{
		v_input--;
		key_b--;
		wait_key(realtime, cur_t_usec, 0.01);
	}

	// limiting values
	v_input = limit(v_input, -MAX_KEYBOARD_COMMAND, MAX_KEYBOARD_COMMAND);

	key_a = limit(key_a, -MAX_KEYBOARD_COMMAND, MAX_KEYBOARD_COMMAND);
	key_b = limit(key_b, -MAX_KEYBOARD_COMMAND, MAX_KEYBOARD_COMMAND);

	// sending references
	uCtrl->set_v_input(v_input);
	uCtrl->set_v_request(MEAN_VELOCITY_REQUEST + SEMI_DIFF_VELOCITY_REQUEST * (v_input / MAX_KEYBOARD_COMMAND));

	uCtrl->set_keyboard_input(key_a, 0, 0);
	uCtrl->set_keyboard_input(key_b, 0, 1);
}

/*! \brief handle inputs comming from joysticks axes
 * 
 * \param[in,out] mbs_data Robotran main structure
 * \param[in] realtime real-time structure
 * \param[in] nb_joysticks number of joysticks detected
 *
 * Use get_Joystick_axis(int joystickID, int axisID, Simu_realtime *realtime)
 * to return the value associated with the joystick number joystickID
 *
 * joystickID: ID of the joystick (starting at 0)
 * axisID: ID of the axis (starting at 0)
 *
 * get_Joystick_axis returns a value in [-1 ; 1] or -10 if this joystickID is not available
 *
 * plugged joysticks are automatically detected at launch (see nb_joysticks)
 *
 * example:
 *     joystick_val = get_Joystick_axis(0, 0, realtime);
 *     mbs_data->user_IO->my_variable = joystick_val * scaling_factor;
 */
void user_joystick_axes(MbsData* mbs_data, Simu_realtime *realtime, int nb_joysticks)
{
	UserCtrl *uCtrl;
	CppInterface *cppInterface;

	cppInterface = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);
	uCtrl = cppInterface->get_ctrl()->get_user_ctrl();

	/*
	// joystick Logitech
	uCtrl->set_joystick_input( get_Joystick_axis(0, 0, realtime), 0);
	uCtrl->set_joystick_input(-get_Joystick_axis(0, 1, realtime), 1);
	uCtrl->set_joystick_input( get_Joystick_axis(0, 3, realtime), 2);
	uCtrl->set_joystick_input(-get_Joystick_axis(0, 4, realtime), 3);//*/

	// joystick Thrustmaster
	uCtrl->set_joystick_input( get_Joystick_axis(0, 0, realtime), 0);
	uCtrl->set_joystick_input(-get_Joystick_axis(0, 1, realtime), 1);
	uCtrl->set_joystick_input( get_Joystick_axis(0, 2, realtime), 2);
	uCtrl->set_joystick_input(-get_Joystick_axis(0, 3, realtime), 3);//*/
}

/*! \brief handle inputs comming from joysticks buttons
 * 
 * \param[in,out] mbs_data Robotran main structure
 * \param[in] buttonID ID of the joystick button
 *
 * buttonID takes a value (usually starting at 0) when a joystick button is pressed
 *
 * Use printf to detect which 'buttonID' is used for the different buttons.
 *
 * example:
 *      if (buttonID == 2) 
 *      {
 *          mbs_data->user_IO->my_command++;
 *      }
 */
void user_joystick_buttons(MbsData* mbs_data, int buttonID)
{

}

#endif
