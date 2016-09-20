#include "LinksRobot.hh"
#include "SimuIndex.hh"
#include "ModelSimuIndex.hh"

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] simu_index indexes of the simulation
 */
LinksRobot::LinksRobot(MbsData *mbs_data, ModelSimuIndex *simu_index, SimuOptions *options)
{
	coman_model = options->coman_model;

	this->mbs_data = mbs_data;
	this->simu_index = simu_index;

	if (coman_model == FLEX_FEET_COMAN)
	{
		// flags
		flag_old = 0; ///< 1 for old values, 0 otherwise
		flag_sec = 0; ///< 1 for second order, 0 otherwise

		flag_cat1 = 1;  ///< 1 for CAT1 (soft), 0 for CAT2 (stiff)
		flag_cosm = 0; 	///< 1 for foot with cosmetic, 0 for foot without cosmetic

		// old values
		if (flag_old)
		{
			// second order
			if (flag_sec)
			{
				// with cosmetic foot
				if (flag_cosm)
				{
					// CAT 1 (soft)
					if (flag_cat1)
					{
						a_heel = 4.314963660026963e6;
						b_heel = 0.012955118726012e6;
						a_toe  = 3.341652522251382e6;
						b_toe  = 0.029679292452165e6;
					}
					// CAT 2 (stiff)
					else
					{
						a_heel = 5.228223330965252e6;
						b_heel = 0.019398049498217e6;
						a_toe  = 3.080794081017493e6;
						b_toe  = 0.034793800965905e6;
					}
				}
				// no cosmetic foot
				else
				{
					// CAT 1 (soft)
					if (flag_cat1)
					{
						a_heel = 0.0;
						b_heel = 6.264595145939355e4;
						a_toe  = 1.005655668236839e7;
						b_toe  = 0.006217655221062e7;
					}
					// CAT 2 (stiff)
					else
					{
						a_heel = 0.0;
						b_heel = 9.139322575131181e4;
						a_toe  = 0.0;
						b_toe  = 9.138591391326264e4;
					}
				}

				k_heel = 0.0;
				k_toe  = 0.0;
			}
			// first order
			else
			{
				// CAT 1 (soft)
				if (flag_cat1)
				{
					k_heel = 62.6e3;
					k_toe  = 97.9e3;
				}
				// CAT 2 (stiff)
				else
				{
					k_heel = 92.0e3;
					k_toe  = 91.4e3;
				}

				a_heel = 0.0;
				b_heel = 0.0;
				a_toe  = 0.0;
				b_toe  = 0.0;
			}

			k_heel_toe = 0.0;
			k_toe_heel = 0.0;
		}
		// new values
		else
		{
			// with cosmetic foot
			if (flag_cosm)
			{
				// CAT 1 (soft)
				if (flag_cat1)
				{
					k_heel_toe = 1.702845611427470e+04;
					k_toe_heel = 1.389189609868269e+04;
				}
				// CAT 2 (stiff)
				else
				{
					k_heel_toe = 2.184278810012924e+04;
					k_toe_heel = 1.965313309554611e+04;
				}
			}
			// no cosmetic foot
			else
			{
				// CAT 1 (soft)
				if (flag_cat1)
				{
					k_heel_toe = 4.599720355909169e+04;
					k_toe_heel = 4.173786050481390e+04;
				}
				// CAT 2 (stiff)
				else
				{
					k_heel_toe = 5.520642032145077e+04;
					k_toe_heel = 6.162201274991522e+04;
				}
			}

			// second order
			if (flag_sec)
			{
				// with cosmetic foot
				if (flag_cosm)
				{
					// CAT 1 (soft)
					if (flag_cat1)
					{
						a_heel = 6.370077305260212e+06;
						b_heel = 6.847393742654708e+03;
						a_toe  = 3.945526025357909e+06;
						b_toe  = 2.848870145737080e+04;
					}
					// CAT 2 (stiff)
					else
					{
						a_heel = 6.816670837280918e+06;
						b_heel = 1.246201677446545e+04;
						a_toe  = 3.753074071033624e+06;
						b_toe  = 3.711614231386696e+04;
					}
				}
				// no cosmetic foot
				else
				{
					// CAT 1 (soft)
					if (flag_cat1)
					{
						a_heel = 4.320162077424030e+06;
						b_heel = 4.978782689908538e+04;
						a_toe  = 1.226563788142112e+07;
						b_toe  = 5.918386158472123e+04;
					}
					// CAT 2 (stiff)
					else
					{
						a_heel = 5.415365612997063e+06;
						b_heel = 8.026999535872451e+04;
						a_toe  = 4.313372200402769e+06;
						b_toe  = 8.498628153579570e+04;
					}
				}
			}
			// first order
			else
			{
				// with cosmetic foot
				if (flag_cosm)
				{
					// CAT 1 (soft)
					if (flag_cat1)
					{
						k_heel = 3.389768878952707e+04;
						k_toe  = 5.180803535671222e+04;
					}
					// CAT 2 (stiff)
					else
					{
						k_heel = 4.340608990486376e+04;
						k_toe  = 5.653310177823145e+04;
					}
				}
				// no cosmetic foot
				else
				{
					// CAT 1 (soft)
					if (flag_cat1)
					{
						k_heel = 6.692041736645819e+04;
						k_toe  = 1.014940268234345e+05;
					}
					// CAT 2 (stiff)
					else
					{
						k_heel = 9.868425512061216e+04;
						k_toe  = 1.012843854587719e+05;
					}
				}
			}
		}

		// natural (resting) spring length
		Z0 = -0.02438;

		// damping
		k_d = 30.0;

		// i/o IDs
		RightFlexDist_id = simu_index->get_mbs_jt(SimuJointIndex::RightFlexDist);
		RightFlexProx_id = simu_index->get_mbs_jt(SimuJointIndex::RightFlexProx);
		LeftFlexDist_id  = simu_index->get_mbs_jt(SimuJointIndex::LeftFlexDist);
		LeftFlexProx_id  = simu_index->get_mbs_jt(SimuJointIndex::LeftFlexProx);
	}
}

/// destructor
LinksRobot::~LinksRobot()
{

}

/*! \brief links of the model computation (interface with C)
 * 
 * \param[in] Z length of the link [m]
 * \param[in] Zd derivative of Z [m/s]
 * \param[in] tsim simulation time [s]
 * \param[in] ilnk ID of the link
 * \return force of the corresponding link [N]
 */
double LinksRobot::compute(double Z, double Zd, double tsim, int ilnk)
{
	double d, d_opp;

	if (coman_model == FLEX_FEET_COMAN)
	{
		// compute the forces
		if ( ilnk == simu_index->get_mbs_link(SimuLinkIndex::RightFlexProx) )
		{
			d = Z - Z0;
			d_opp = mbs_data->q[RightFlexDist_id] - Z0;

			if (flag_sec)
			{
				return a_heel*d*d + b_heel*d + k_d*Zd + k_heel_toe*d_opp;
			}
			else
			{
				return k_heel*d + k_d*Zd + k_heel_toe*d_opp;
			}			
		}
		else if ( ilnk == simu_index->get_mbs_link(SimuLinkIndex::RightFlexDist) )
		{
			d = Z - Z0;
			d_opp = mbs_data->q[RightFlexProx_id] - Z0;

			if (flag_sec)
			{
				return a_toe*d*d + b_toe*d + k_d*Zd + k_toe_heel*d_opp;
			}
			else
			{
				return k_toe*d + k_d*Zd + k_toe_heel*d_opp;
			}			
		}
		else if ( ilnk == simu_index->get_mbs_link(SimuLinkIndex::LeftFlexProx) )
		{
			d = Z - Z0;
			d_opp = mbs_data->q[LeftFlexDist_id] - Z0;

			if (flag_sec)
			{
				return a_heel*d*d + b_heel*d + k_d*Zd + k_heel_toe*d_opp;
			}
			else
			{
				return k_heel*d + k_d*Zd + k_heel_toe*d_opp;
			}			
		}
		else if ( ilnk == simu_index->get_mbs_link(SimuLinkIndex::LeftFlexDist) )
		{
			d = Z - Z0;
			d_opp = mbs_data->q[LeftFlexProx_id] - Z0;

			if (flag_sec)
			{
				return a_toe*d*d + b_toe*d + k_d*Zd + k_toe_heel*d_opp;
			}
			else
			{
				return k_toe*d + k_d*Zd + k_toe_heel*d_opp;
			}
		}

		return  0.0;
	}
}
