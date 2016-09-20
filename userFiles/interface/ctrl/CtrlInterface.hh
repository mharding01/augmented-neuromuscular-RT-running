/*! 
 * \author Nicolas Van der Noot
 * \file CtrlInterface.hh
 * \brief CtrlInterface class
 * 
 * Details
 */

#ifndef _CTRL_INTERFACE_HH_
#define _CTRL_INTERFACE_HH_

#include "CppInterface.hh"
#include "JointsInit.hh"

/*! \brief c++ interface to choose the controller
 */
class CtrlInterface: public CppInterface
{
	public:
		CtrlInterface(MbsData *mbs_data);
		virtual ~CtrlInterface();
	private:
		JointsInit *joints_init;
};

#endif
