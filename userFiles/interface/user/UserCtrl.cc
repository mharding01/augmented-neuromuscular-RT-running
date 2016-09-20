
#include "UserCtrl.hh"

/*! \brief constructor
 */
UserCtrl::UserCtrl()
{
	v_input   = 0;
	v_request = 0.6;

	for(int i=0; i<2; i++)
	{
		for(int j=0; j<2; j++)
		{
			keyboard_input[i][j] = 0;
		}
	}

	for(int i=0; i<4; i++)
	{
		joystick_input[i] = 0.0;
	}
}

/*! \brief destrcutor
 */
UserCtrl::~UserCtrl()
{

}
