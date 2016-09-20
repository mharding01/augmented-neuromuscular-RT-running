
#include "Vec3DSimu.hh"

/*! \brief constructor
 */
Vec3DSimu::Vec3DSimu()
{
	for(int i=0; i<3; i++)
	{
		vec[i] = 0.0;
	}
}

/*! \brief constructor
 *
 * \param[in] x first vector element
 * \param[in] y second vector element
 * \param[in] z third vector element
 */
Vec3DSimu::Vec3DSimu(double x, double y, double z)
{
	vec[0] = x;
	vec[1] = y;
	vec[2] = z;
}

/*! \brief destructor
 */
Vec3DSimu::~Vec3DSimu()
{

}
