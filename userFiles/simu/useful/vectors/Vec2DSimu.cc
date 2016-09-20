
#include "Vec2DSimu.hh"

/*! \brief constructor
 */
Vec2DSimu::Vec2DSimu()
{
	for(int i=0; i<2; i++)
	{
		vec[i] = 0.0;
	}
}

/*! \brief constructor
 *
 * \param[in] x first vector element
 * \param[in] y second vector element
 */
Vec2DSimu::Vec2DSimu(double x, double y)
{
	vec[0] = x;
	vec[1] = y;
}

/*! \brief destructor
 */
Vec2DSimu::~Vec2DSimu()
{

}
