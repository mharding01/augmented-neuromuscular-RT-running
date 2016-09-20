
#include "DelayGeyer.hh"

// delay parameters
#define DELAY_L 0.01   ///< [ms] 0.02  in HG paper (2010) for ankle
#define DELAY_M 0.005  ///< [ms] 0.01  in HG paper (2010) for vas and ground
#define DELAY_S 0.0025 ///< [ms] 0.005 in HG paper (2010) for hip

#define DELAY_SIZE_L 10 ///< size of the large delay tab [-]
#define DELAY_SIZE_M 5  ///< size of the medium delay tab [-]
#define DELAY_SIZE_S 3  ///< size of the small delay tab [-]

/*! \brief constructor
 */
DelayGeyer::DelayGeyer()
{
	for(int i=0; i<DELAY_GEYER_NB; i++)
	{
		switch (i)
		{
			// feet forces
			case F_FOOT_R : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case F_FOOT_L : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;

			// angles
			case PHI_K_R  : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case PHI_K_L  : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case PHIP_K_R : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case PHIP_K_L : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case PHI_H_R  : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case PHI_H_L  : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case PHIP_H_R : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case PHIP_H_L : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;

			// trunk
			case THETA_TORSO : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case OMEGA_TORSO : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;

			// l.ce
			case LCE_TA_R  : delay_tab.push_back(new Delay(DELAY_L, DELAY_SIZE_L)); break;
			case LCE_TA_L  : delay_tab.push_back(new Delay(DELAY_L, DELAY_SIZE_L)); break;
			case LCE_VAS_R : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case LCE_VAS_L : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case LCE_HAM_R : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case LCE_HAM_L : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case LCE_HFL_R : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case LCE_HFL_L : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;

			// F.m
			case F_SOL_R : delay_tab.push_back(new Delay(DELAY_L, DELAY_SIZE_L)); break;
			case F_SOL_L : delay_tab.push_back(new Delay(DELAY_L, DELAY_SIZE_L)); break;
			case F_VAS_R : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case F_VAS_L : delay_tab.push_back(new Delay(DELAY_M, DELAY_SIZE_M)); break;
			case F_GAS_R : delay_tab.push_back(new Delay(DELAY_L, DELAY_SIZE_L)); break;
			case F_GAS_L : delay_tab.push_back(new Delay(DELAY_L, DELAY_SIZE_L)); break;
			case F_HAM_R : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case F_HAM_L : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case F_GLU_R : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
			case F_GLU_L : delay_tab.push_back(new Delay(DELAY_S, DELAY_SIZE_S)); break;
		
			default:
				break;
		}
	}
}

/*! \brief destructor
 */
DelayGeyer::~DelayGeyer()
{
	// delete already done in parent class
}
