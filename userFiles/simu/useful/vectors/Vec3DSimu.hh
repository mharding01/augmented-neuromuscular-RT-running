/*! 
 * \author Nicolas Van der Noot
 * \file Vec3DSimu.hh
 * \brief Vec3DSimu class
 */

#ifndef _VEC_3D_SIMU_
#define _VEC_3D_SIMU_

#include <cmath>

/*! \brief 3D vectors
 */
class Vec3DSimu
{
	public:
		Vec3DSimu();
		Vec3DSimu(double x, double y, double z);
		~Vec3DSimu();

		/*! \brief get a vector element
		 * 
		 * \pre index is in the [0;2] interval
		 * \param[in] index of the requested component
		 * \return distance between the two points [m]
		 */
		inline double get(int index) const { return vec[index]; }

		/*! \brief set a component of the vector
		 * 
		 * \pre index is in the [0;2] interval
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
		 * \param[in] z third vector element
		 */
		inline void set_val(double x, double y, double z)
		{
			vec[0] = x;
			vec[1] = y;
			vec[2] = z;
		}

		/*! \brief get the norm
		 * 
		 * \return vector norm
		 */
		inline double get_norm()
		{
			return sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
		}


	private:
		double vec[3];
};

/*! \brief sum two vectors with the + operation
 * 
 * \param[in] a first vector to sum
 * \param[in] b second vector to sum 
 * \return sum of the two vectors
 */
inline Vec3DSimu operator+(Vec3DSimu const& a, Vec3DSimu const& b)
{
	Vec3DSimu result;

	for (int i=0; i<3; i++)
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
inline Vec3DSimu operator-(Vec3DSimu const& a, Vec3DSimu const& b)
{
	Vec3DSimu result;

	for (int i=0; i<3; i++)
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
inline Vec3DSimu operator*(double const& a, Vec3DSimu const& b)
{
	Vec3DSimu result;

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
inline Vec3DSimu operator*(Vec3DSimu const& a, double const& b)
{
	Vec3DSimu result;

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
inline Vec3DSimu operator/(Vec3DSimu const& a, double const& b)
{
	Vec3DSimu result;

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
inline Vec3DSimu get_nul_vec_3D()
{
	Vec3DSimu result(0.0, 0.0, 0.0);

	return result;
}

/*! \brief return a vector
 * 
 * \param[in] x x component
 * \param[in] y y component
 * \param[in] z z component
 * \return vector with x, y and z components
 */
inline Vec3DSimu get_vec_3D(double x, double y, double z)
{
	Vec3DSimu result(x, y, z);

	return result;
}

#endif
