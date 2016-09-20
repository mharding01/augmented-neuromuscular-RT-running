/*! 
 * \author Nicolas Van der Noot
 * \file sensors_get.h
 * \brief get sensors info for C files (C++ interface)
 */

#ifndef _SENSORS_GET_H_
#define _SENSORS_GET_H_

#include "mbs_data.h"

double get_S_MidWaist_R(MbsData *mbs_data, int index);
double get_S_MidWaist_OM(MbsData *mbs_data, int index);
double get_S_MidWaist_OMP(MbsData *mbs_data, int index);
double get_S_MidWaist_P(MbsData *mbs_data, int index);

double get_S_RFoots_R(MbsData *mbs_data, int index);
double get_S_RFoots_P(MbsData *mbs_data, int index);
double get_S_LFoots_R(MbsData *mbs_data, int index);
double get_S_LFoots_P(MbsData *mbs_data, int index);

double get_S_RWrist_P(MbsData *mbs_data, int index);
double get_S_LWrist_P(MbsData *mbs_data, int index);
double get_S_Ball_P(MbsData *mbs_data, int index);

double get_S_RWrist_V(MbsData *mbs_data, int index);
double get_S_LWrist_V(MbsData *mbs_data, int index);
double get_S_Ball_V(MbsData *mbs_data, int index);

double get_waist_to_feet(MbsData *mbs_data);

#endif
