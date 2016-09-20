/*! 
 * \author Nicolas Van der Noot
 * \file LinksRobot.hh
 * \brief LinksRobot
 */

#ifndef _LINKS_ROBOT_HH_
#define _LINKS_ROBOT_HH_

#include "SimuOptions.h"
#include "ModelSimuIndex.hh"
#include "mbs_data.h"

/*! \brief all the robot links
 */
class LinksRobot
{
	public:
		LinksRobot(MbsData *mbs_data, ModelSimuIndex *simu_index, SimuOptions *options);
		~LinksRobot();

		double compute(double Z, double Zd, double tsim, int ilnk);

	private:
		MbsData *mbs_data; ///< Robotran structure
		ModelSimuIndex *simu_index; ///< simulation indexes

		double a_heel; ///< second order: heel, first coefficient [N/m^2]
		double a_toe;  ///< second order: toe , first coefficient [N/m^2]
		double b_heel; ///< second order: heel, second coefficient[N/m]
		double b_toe;  ///< second order: toe , second coefficient[N/m]

		double k_heel; ///< stiffness: heel [N/m^2]
		double k_toe;  ///< stiffness: toe [N/m^2]
		double k_heel_toe; ///< stiffness: cross effect of the toe on the heel [N/m^2]
		double k_toe_heel; ///< stiffness: cross effect of the heel on the toe [N/m^2]

		double k_d; ///< damping coefficient [N s/m]

		double Z0; ///< natural (resting) spring length [m]

		int coman_model; ///< COMAN model

		int flag_old;  ///< 1 for old values, 0 otherwise
		int flag_sec;  ///< 1 for second order, 0 otherwise
		int flag_cat1; ///< 1 for CAT1 (soft), 0 for CAT2 (stiff)
		int flag_cosm; ///< 1 for foot with cosmetic, 0 for foot without cosmetic

		// i/o IDs
		int RightFlexDist_id; ///< right flexible foot, distal part (toe)
		int RightFlexProx_id; ///< right flexible foot, proximal part (heel)
		int LeftFlexDist_id;  ///< left flexible foot, distal part (toe)
		int LeftFlexProx_id;  ///< left flexible foot, proximal part (heel)
};

#endif
