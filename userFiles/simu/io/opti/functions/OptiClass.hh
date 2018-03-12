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

		void set_stride_period_mean(double value) { stride_period_mean = value; }
		void set_stride_length_mean(double value) { stride_length_mean = value; }
		void set_take_off_mean(double value)      { take_off_mean = value;      }
		void set_ds_cycle_mean(double value)      { ds_cycle_mean = value;      }
		void set_flight_cycle_mean(double value)  { flight_cycle_mean = value;  }

		void set_met_energy_legs(double value)  { met_energy_legs = value;  }
		void set_met_energy_total(double value) { met_energy_total = value; }

		void set_t_final(double value) { t_final = value; }

		double get_v_ref() const { return v_ref; }
		double get_fitness() const { return fitness; }
		int get_size() const { return optiNorms.size(); }
		std::vector<double> get_param() const { return optiParams; }
		std::vector<double> get_fitness_details() { return fitness_details; }

		double get_v_real() const { return v_real; }

		double get_stride_period_mean() const { return stride_period_mean; }
		double get_stride_length_mean() const { return stride_length_mean; }
		double get_take_off_mean() const      { return take_off_mean;      }
		double get_ds_cycle_mean() const      { return ds_cycle_mean;      }
		double get_flight_cycle_mean() const  { return flight_cycle_mean;  }

		double get_met_energy_legs() const  { return met_energy_legs;  }
		double get_met_energy_total() const { return met_energy_total; }

		double get_t_final() const { return t_final; }

		void reset();

	private:
		std::vector<double> optiNorms;  ///< optimization parameters (normalized in [0;1])
		std::vector<double> optiParams; ///< real optimization parameters

		double fitness; ///< fitness function
		std::vector<double> fitness_details; ///< details of fitness stage results 

		double v_ref; ///< speed reference [m/s]

		double v_real; ///< real speed [m/s]

		double stride_period_mean; ///< mean of the stride period [s]
		double stride_length_mean; ///< mean of the stride length [m]
		double take_off_mean;      ///< mean for starting take off [%]
		double ds_cycle_mean;      ///< mean of the double support per cycle [%]
		double flight_cycle_mean;  ///< mean of the flight phase per cycle [%]

		double met_energy_legs;  ///< metabolic energy for the sagittal leg muscles
		double met_energy_total; ///< metabolic energy for all the muscles

		double t_final; ///< final time
};

#endif
