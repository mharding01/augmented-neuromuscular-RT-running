/*! 
 * \author Nicolas Van der Noot
 * \file coman_properties.hh
 * \brief properties of the COMAN
 */

#ifndef _COMAN_PROPERTIES_HH_
#define _COMAN_PROPERTIES_HH_

#define GRAVITY 9.81       ///< gravity [m/s^2]
#define MASS_COMAN 28.3574 ///< mass of the COMAN robot [kg]
#define WEIGHT_COMAN (MASS_COMAN * GRAVITY) ///< weight of the COMAN [N]
#define LEG_LENGTH 0.427

#define NB_MOTORS 23 ///< number of motors

enum{R_ID, L_ID, NB_LEGS};

#endif
