/*! 
 * \author Nicolas Van der Noot
 * \file Vec2DSimu.hh
 * \brief Vec2DSimu class
 */

#ifndef _VEC_2D_SIMU_
#define _VEC_2D_SIMU_

#include <cmath>

/*! \brief 2D vectors
 */
class Vec2DSimu
{
	public:
		Vec2DSimu();
		Vec2DSimu(double x, double y);
		~Vec2DSimu();

		/*! \brief get a vector element
		 * 
		 * \pre index is in the [0;1] interval
		 * \param[in] index of the requested component
		 * \return distance between the two points [m]
		 */
		inline double get(int index) const { return vec[index]; }

		/*! \brief set a component of the vector
		 * 
		 * \pre index is in the [0;1] interval
		 * \param[in] index index of the component 
		 * \param[in] value new value to set
		 */
		inline void set(int index, double value)
		{
			vec[index] = value;
		}

		/*! \brief set x and y components of the vector
		 * 
		 * \param[in] x first vector element
		 * \param[in] y second vector element
		 */
		inline void set_val(double x, double y)
		{
			vec[0] = x;
			vec[1] = y;
		}

		/*! \brief get the norm
		 * 
		 * \return vector norm
		 */
		inline double get_norm()
		{
			return sqrt(vec[0]*vec[0] + vec[1]*vec[1]);
		}

	private:
		double vec[2];
};

/*! \brief sum two vectors with the + operation
 * 
 * \param[in] a first vector to sum
 * \param[in] b second vector to sum 
 * \return sum of the two vectors
 */
inline Vec2DSimu operator+(Vec2DSimu const& a, Vec2DSimu const& b)
{
	Vec2DSimu result;

	for (int i=0; i<2; i++)
	{
		result.set(i, a.get(i) + b.get(i));
	}

	return result;
}

/*! \brief difference of two vectors with the - opration
 * 
 * \param[in] a first vector
 * \param[in] b second vector 
 * \return difference of the two vectors
 */
inline Vec2DSimu operator-(Vec2DSimu const& a, Vec2DSimu const& b)
{
	Vec2DSimu result;

	for (int i=0; i<2; i++)
	{
		result.set(i, a.get(i) - b.get(i));
	}

	return result;
}

/*! \brief multiply a vector by a coefficient
 * 
 * \param[in] a coefficient
 * \param[in] b vector
 * \return multiplication: a*b
 */
inline Vec2DSimu operator*(double const& a, Vec2DSimu const& b)
{
	Vec2DSimu result;

	for (int i=0; i<3; i++)
	{
		result.set(i, a * b.get(i));
	}

	return result;
}

/*! \brief multiply a vector by a coefficient
 * 
 * \param[in] a vector
 * \param[in] b coefficient
 * \return multiplication: a*b
 */
inline Vec2DSimu operator*(Vec2DSimu const& a, double const& b)
{
	Vec2DSimu result;

	for (int i=0; i<3; i++)
	{
		result.set(i, a.get(i) * b);
	}

	return result;
}

/*! \brief division of a vector by a coefficient
 * 
 * \param[in] a vector
 * \param[in] b coefficient
 * \return division: a/b
 */
inline Vec2DSimu operator/(Vec2DSimu const& a, double const& b)
{
	Vec2DSimu result;

	for (int i=0; i<3; i++)
	{
		result.set(i, a.get(i) / b);
	}

	return result;
}

/*! \brief return a vector with only zero values
 * 
 * \return vector
 */
inline Vec2DSimu get_nul_vec_2D()
{
	Vec2DSimu result(0.0, 0.0);

	return result;
}

/*! \brief return a vector
 * 
 * \param[in] x x component
 * \param[in] y y component
 * \return vector with x and y components
 */
inline Vec2DSimu get_vec_2D(double x, double y)
{
	Vec2DSimu result(x, y);

	return result;
}

#endif
