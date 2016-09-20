#include "Actuator.hh"
#include "user_IO.h"

#define MAX_ACT_VOLTAGE 100.0  ///< max voltage for the actuators [V]

#define MAX_NOISE_TOR 0.4     ///< max noise torque [Nm]
#define PERIOD_NOISE_TOR 0.01 ///< period for the torque noise [s]

inline double limit(double x, double min, double max){ return (x < min) ? min : (x > max) ? max : x; }

inline double sgn(double x){ return (x < 0.0) ? -1.0 : (x>0.0) ? 1.0 : 0.0; }

/*! \brief constructor
 * 
 * \param[in] mbs_data Robotran structure
 * \param[in] simu_index simulation indexes list
 * \param[in] index index of the joint (simulation list)
 * \param[in] K_s passive stiffness [Nm/rad]
 * \param[in] D_s passive damping [Nm*s/rad]
 * \param[in] J motor inertia [kg*m^2]
 * \param[in] D motor back-EMF constant and the rotor friction [Nm*s/rad]
 * \param[in] V_T voltage to torque ratio [Nm/V]
 * \param[in] Aco friction model step factor [Nm]
 * \param[in] Bco friction model proportional factor [Nm*s/rad]
 */
Actuator::Actuator(MbsData *mbs_data, ModelSimuIndex *simu_index, int index, double K_s, double D_s, double J, double D, double V_T, double Aco, double Bco)
{
	// mbs_data info
	this->mbs_data   = mbs_data;
	this->index = index;

	mbs_index = simu_index->get_mbs_jt(index);

	// joint kinematics
	q  = mbs_data->q[mbs_index];
	qd = mbs_data->qd[mbs_index];

	// motor kinematics
	q_m   = q;
	qd_m  = qd;
	qdd_m = 0.0;

	// torque
	Qq = 0.0;

	// motor characteristics
	this->K_s = K_s;
	this->D_s = D_s;

	this->J = J;
	this->D = D;

	this->V_T = V_T;

	J_inv = 1.0 / J;

	// friction model
	this->Aco = Aco;
	this->Bco = Bco;

	// voltage
	u = 0.0;

	// position low-level controller gains
	P_stiffness = 0.0;
	D_damping   = 0.0;

	// torque low-level controller gains
	P = 0.0;
	I = 0.0;

	// references
	q_ref  = 0.0;
	Qq_ref = 0.0;

	// integral computation
	old_err_Qq = 0.0;
	int_err_Qq = 0.0;
	last_t_err_Qq = mbs_data->tsim;

	torque_noise = new NoiseSimu(mbs_data->tsim, MAX_NOISE_TOR, PERIOD_NOISE_TOR);

	options = mbs_data->user_IO->options;
	torque_noisee = options->torque_noise;
	ctrl_two_parts = options->ctrl_two_parts;
}

/*! \brief desctructor
 */
Actuator::~Actuator()
{
	delete torque_noise;
}

/*! \brief set the position and velocity of the joint and its motor
 * 
 * \param[in] q_m motor position [rad]
 * \param[in] qd_m motor veocity [rad/s]
 */
void Actuator::set_pos_vel(double q_m, double qd_m)
{
	q  = mbs_data->q[mbs_index];
	qd = mbs_data->qd[mbs_index];

	this->q_m  = q_m;
	this->qd_m = qd_m;
}

/*! \brief set the stiffness and damping values of the position controller
 * 
 * \param[in] P_stiffness proportional gain = stiffness [Nm/rad]
 * \param[in] D_damping derivative gain = damping [(Nm s)/rad]
 */
void Actuator::set_stiffness_damping(double P_stiffness, double D_damping)
{
	this->P_stiffness = P_stiffness;
	this->D_damping   = D_damping;
}

/*! \brief set the PI gains of the torque controller
 * 
 * \param[in] P proportional gain [V/Nm]
 * \param[in] I integral gain [V/(Nm s)]
 */
void Actuator::set_PI_gains(double P, double I)
{
	this->P = P;
	this->I = I;
}

/*! \brief compute the motor acceleration
 *
 * J*qdd_m + D*qd_m + Qq = V_T*u
 * Qq = K_s*(q_m-q) + D_s*(qd_m-qd) 
 *
 * qdd_m = (1/J) * (V_T*u - D*qd_m - Qq)
 * qdd_m = (1/J) * (V_T*u - D*qd_m - K_s*(q_m-q) - D_s*(qd_m-qd))
 */
void Actuator::compute_acceleration()
{
	// motor model
	qdd_m = J_inv * ( V_T*u - D*qd_m - K_s*(q_m-q) - D_s*(qd_m-qd) );

	// friction model
	if (options->joints_friction)
	{
		qdd_m += J_inv * (Aco*sgn(qd) + Bco*qd);
	}
}

/*! \brief compute the motor torque
 *
 * J*qdd_m + D*qd_m + Qq = V_T*u
 * Qq = K_s*(q_m-q) + D_s*(qd_m-qd) 
 *
 * qdd_m = (1/J) * (V_T*u - D*qd_m - Qq)
 * qdd_m = (1/J) * (V_T*u - D*qd_m - K_s*(q_m-q) - D_s*(qd_m-qd))
 */
void Actuator::compute_torque()
{
	Qq = K_s*(q_m-q) + D_s*(qd_m-qd);
}

/*! \brief low-level impedance controller to compute the motor voltage
 */
void Actuator::low_level_controller()
{
	double t;
	double tor_ref, err_Qq;

	t = mbs_data->tsim;

	tor_ref = P_stiffness * (q_ref - q_m) - D_damping * qd_m + Qq_ref;

	err_Qq = tor_ref - Qq;

	if ( (torque_noisee && !ctrl_two_parts) || (torque_noisee && ctrl_two_parts && (t > 5.087)) ) //!\\ time had coded, must to be sup born of t_switch
	{
		err_Qq += torque_noise->update_and_get(t);
	}

	int_err_Qq += 0.5 * (old_err_Qq + err_Qq) * (t - last_t_err_Qq);

	last_t_err_Qq = t;
	old_err_Qq    = err_Qq;

	u = P * err_Qq + I * int_err_Qq;

	u = limit(u, -MAX_ACT_VOLTAGE, MAX_ACT_VOLTAGE);
}
