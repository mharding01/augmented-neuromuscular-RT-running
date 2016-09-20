
#include "Ctrl.hh"

/*! \brief constructor
 */
Ctrl::Ctrl(int ctrl_id)
{
	this->ctrl_id = ctrl_id;
	
	user_ctrl = new UserCtrl();
}

/*! \brief destructor
 */
Ctrl::~Ctrl()
{
	delete user_ctrl;
}
