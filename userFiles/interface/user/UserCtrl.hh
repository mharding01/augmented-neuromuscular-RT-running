/*! 
 * \author Nicolas Van der Noot
 * \file UserCtrl.hh
 * \brief UserCtrl class
 */

#ifndef _USER_CTRL_HH_
#define _USER_CTRL_HH_

#include <iostream>

/*! \brief User commands to control the robot
 */
class UserCtrl
{
	public:
		UserCtrl();
		~UserCtrl();

		int get_v_input() const { return v_input; }
		double get_v_request() const { return v_request; }
		int get_keyboard_input(int i, int j) const { return keyboard_input[i][j]; }
		double get_joystick_input(int i) const { return joystick_input[i]; }


		void set_v_input(int value) { v_input = value; }
		void set_v_request(double value) { v_request = value; }
		void set_keyboard_input(int value, int i, int j) { keyboard_input[i][j] = value; }
		void set_joystick_input(double value, int i) { joystick_input[i] = value; }

	private:
		int v_input; ///< input from the keyboard
		double v_request; ///< requested speed (for cpg adapt speed)

		int keyboard_input[2][2];    ///< arrows keyboard or (Z,Q,S,D or W,A,S,D)
		double joystick_input[4]; ///< joystick handle
};

#endif
