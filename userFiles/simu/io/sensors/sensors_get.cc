
#include "sensors_get.h"
#include "CppInterface.hh"
#include "SensorsInfo.hh"
#include "user_model.h"

/*! \brief get SensorsInfo from mbs_data
 * 
 * \param[in] mbs_data Robotran structure
 * \return SensorsInfo class
 */
SensorsInfo* get_sensors_info(MbsData *mbs_data)
{
	CppInterface *cpp_int = static_cast<CppInterface*>(mbs_data->user_model->cppInterface);

	return static_cast<SensorsInfo*>(cpp_int->get_simu_ctrl()->get_computation(SENSORS_INFO));
}

/*! \brief get S_MidWaist_R
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_MidWaist_R(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_MidWaist_R(index);
}

/*! \brief get S_MidWaist_OM
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_MidWaist_OM(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_MidWaist_OM(index);
}

/*! \brief get S_MidWaist_OMP
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_MidWaist_OMP(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_MidWaist_OMP(index);
}

/*! \brief get S_MidWaist_P
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_MidWaist_P(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_MidWaist_P(index);
}

/*! \brief get S_RFoots_R
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_RFoots_R(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_RFoots_R(index);
}

/*! \brief get S_RFoots_P
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_RFoots_P(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_RFoots_P(index);
}

/*! \brief get S_LFoots_R
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_LFoots_R(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_LFoots_R(index);
}

/*! \brief get S_LFoots_P
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_LFoots_P(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_LFoots_P(index);
}

/*! \brief get S_RWrist_P
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_RWrist_P(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_RWrist_P(index);
}

/*! \brief get S_LWrist_P
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_LWrist_P(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_LWrist_P(index);
}

/*! \brief get S_Ball_P
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_Ball_P(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_Ball_P(index);
}

/*! \brief get S_RWrist_V
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_RWrist_V(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_RWrist_V(index);
}

/*! \brief get S_LWrist_V
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_LWrist_V(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_LWrist_V(index);
}

/*! \brief get S_Ball_V
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] index index of the element
 * \return requested sensor value
 */
double get_S_Ball_V(MbsData *mbs_data, int index)
{
	return get_sensors_info(mbs_data)->get_S_Ball_V(index);
}

/*! \brief get waist_to_feet
 * 
 * \param[in] mbs_data Robotran structure
 * \return requested sensor value
 */
double get_waist_to_feet(MbsData *mbs_data)
{
	return get_sensors_info(mbs_data)->get_waist_to_feet();
}
