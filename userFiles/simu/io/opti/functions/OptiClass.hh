/*! 
 * \author Nicolas Van der Noot
 * \file OptiClass.hh
 * \brief OptiClass class
 */

#ifndef _OPTI_CLASS_HH_
#define _OPTI_CLASS_HH_

#include <vector>

/*! \brief Optimization IO class structure
 */
class OptiClass
{
	public:
		OptiClass();
		~OptiClass();

		void add_param(double normValue);

		void set_v_ref(double value) { v_ref = value; }
		void set_fitness(double value) { fitness = value; }
		void set_fitness_details(std::vector<double> values) { fitness_details = values; }

		void set_v_real(double value) { v_real = value; }

		double get_v_ref() const { return v_ref; }
		double get_fitness() const { return fitness; }
		int get_size() const { return optiNorms.size(); }
		std::vector<double> get_param() const { return optiParams; }
		std::vector<double> get_fitness_details() { return fitness_details; }

		double get_v_real() const { return v_real; }

		void reset();

	private:
		std::vector<double> optiNorms;  ///< optimization parameters (normalized in [0;1])
		std::vector<double> optiParams; ///< real optimization parameters

		double fitness; ///< fitness function
		std::vector<double> fitness_details; ///< details of fitness stage results 

		double v_ref; ///< speed reference [m/s]

		double v_real; ///< real speed [m/s]
};

#endif
