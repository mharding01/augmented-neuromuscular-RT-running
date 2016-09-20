/*! 
 * \author Nicolas Van der Noot
 * \file CppInterface.hh
 * \brief CppInterface class
 */

#ifndef _CPP_INTERFACE_HH_
#define _CPP_INTERFACE_HH_

extern "C" {
	#include "mbs_data.h"
}

#include "Ctrl.hh"
#include "SimuCtrl.hh"

/*! \brief Main file for the interface with the C++
 */
class CppInterface
{
	public:
		CppInterface(MbsData *mbs_data);
		virtual ~CppInterface();

		/// get ctrl
		inline Ctrl* get_ctrl() { return ctrl; }

		/// get simu_ctrl
		inline SimuCtrl* get_simu_ctrl() { return simu_ctrl; }

	protected:
		MbsData *mbs_data; ///< Robotran structure
		Ctrl *ctrl;       ///< controller
		SimuCtrl *simu_ctrl;    ///< simulation and link with the controller gestion
};

#endif
