
#include "Muscle.hh"
#include "StimulationCtrl.hh"
#include "ctrl_functions.hh"

#include <cmath>

#define MAX_NOISE 0.05       ///< maximal noise level on the msucles [-]
#define NOISE_PERIOD 0.1     ///< noise period for the muscles [s]

#define PI_2 (M_PI / 2.0)

#define TAU_FILTER 0.01

#define K_MUSCLE 5.0 ///< shape factor
#define N_MUSCLE 1.5 ///< force enhancement

#define CM_TO_M 0.01

#define F_L_INF 0.001 ///< lower bound

#define F_V_INF 0.0
#define F_V_SUP 1.5

#define W_MUSCLE 0.56        ///< width (portion of l_opt)
#define C_MUSCLE (log(0.05)) ///< residual force factor

#define EPSILON_REF 0.04
#define EPSILON_BE (W_MUSCLE / 2.0)
#define EPSILON_PE W_MUSCLE

#define THRES_L_CE_CONV 1.0e-10
#define NB_MAX_L_CE_CONV 10

#define NB_EULER_ITER 5

/// get square
inline double square(double x) { return x*x; }

/// get cube
inline double cube(double x) { return x*x*x; }

/// positive value or 0
inline double pos(double x){ return (x > 0.0) ?  x : 0.0; }

/*! \brief constructor
 * 
 * \param[in] inputs inputs of the controller
 * \param[in] muscle_id ID of the muscle
 * \param[in] body_part_id ID of the body part
 * \param[in] l_opt optimal length [m]
 * \param[in] l_slack salck length [m]
 * \param[in] F_max maximal force [N]
 * \param[in] v_max max velocity [l_opt/s] (! positive)
 * \param[in] m_mtu mass of the muscle [kg]
 * \param[in] t_I mass fraction of slow twitch fibers [-]
 */
Muscle::Muscle(CtrlInputs *inputs, int muscle_id, int body_part_id, double l_opt, double l_slack, double F_max, double v_max, double m_mtu, double t_I)
{
	this->inputs = inputs;

	this->muscle_id = muscle_id;
	this->body_part_id = body_part_id;

	muscle_noise = inputs->get_options()->is_muscle_noise();
	ctrl_two_parts = inputs->get_options()->is_ctrl_two_parts();

	p.l_opt   = l_opt;
	p.l_slack = l_slack;
	p.l_min   = (1.0 - W_MUSCLE) * l_opt;
	p.F_max   = F_max;
	p.v_max   = v_max;
	p.m_mtu   = m_mtu;
	p.t_I     = t_I;

	v.ce = 0.0;
	v.ce_norm = 0.0;

	f.v = 0.0;
	f.l = 0.0;

	F.se      = 0.0;
	F.be      = 0.0;
	F.pe_star = 0.0;
	F.ce      = 0.0;
	F.m       = 0.0;

	A = 0.0;
	S = 0.0;

	// muscle inverse constants
	c.K1 = p.F_max  / square(EPSILON_REF * p.l_slack);
	c.K2 = p.F_max  / square(EPSILON_BE  * p.l_opt);
	c.K3 = p.F_max  / square(EPSILON_PE  * p.l_opt);
	c.K4 = C_MUSCLE / cube(  W_MUSCLE    * p.l_opt);
	c.K5 = pow( (1/c.K4) * log(F_L_INF) , (1.0/3.0) );
	c.K6 = 7.56 * K_MUSCLE;
	c.K7 = 1.0 - (c.K6 + 1.0) * N_MUSCLE;

	n = new Noise(inputs->get_t(), MAX_NOISE, NOISE_PERIOD);

	A_filter = new LowFilter(TAU_FILTER, S, inputs->get_t());

	last_t     = inputs->get_t();
	last_t_inv = inputs->get_t();

	met_energy = 0.0;

	// initial lengths
	l.mtu = l_opt + l_slack;

	l.ce = l_opt;

	l.se  = l.mtu - l.ce;

	for(int i=0; i<L_CE_FINITE_NB; ++i)
	{
		prev_lce[i] = 0.0;
	}

	// finite difference order 3 coeffcients
	coef_diff_1 = -1.0 / 3.0;
	coef_diff_2 =  1.5;
	coef_diff_3 = -3.0;
	coef_diff_4 =  11.0 / 6.0;
}

/*! \brief destructor
 */
Muscle::~Muscle()
{
	delete n;
	delete A_filter;
}

/*! \brief main computations
 */
void Muscle::compute(StimulationCtrl *stim_ctrl)
{
	double cur_t, diff_t;

	// time
	cur_t  = inputs->get_t();
	diff_t = cur_t - last_t;

	// safety
	if (diff_t <= 0.0) { return; }

	last_t = cur_t;

	// stimulation
	S = stim_ctrl->get_Stim(body_part_id, muscle_id);

	// adding noise
	if ( (muscle_noise && !ctrl_two_parts) || (muscle_noise && ctrl_two_parts && (cur_t > 5.087)) ) //!\\ time had coded, must to be sup born of t_switch
	{
		S *= (1.0 + n->update_and_get(cur_t));
	}

	S = limit_range(S, S_MIN, S_MAX);

	// activation from stimulation
	A = A_filter->update_and_get(S, cur_t);

	// kinematics
	rm_compute();
	lmtu_compute();

	// integration
	euler_iterations(diff_t, NB_EULER_ITER);

	// torques
	torques_compute();

	// energy
	met_energy += metabolic_energy_dot() * diff_t;

	// save timing
	last_t = cur_t;
}

/*! \brief force computation
 *
 * \param[in] cur_l_ce current value of 'l_ce' [m]
 */
void Muscle::forces_compute(double cur_l_ce)
{
	// --- Variables declaration --- //
	
	double epsilon;
	double f_see;
	
	// --- l_se --- //
	
	l.se = l.mtu - cur_l_ce;
	
	
	// --- F_se --- //
	// F_se = F_max * f_see
	// f_see = (epsilon/epsilon_ref)^2 -> HG 2003
	// epsilon = (l_se - l_rest)/l_rest -> l_rest = l_slack

	// epsilon
	epsilon = (l.se - p.l_slack) / p.l_slack;
	
	// f_see
	f_see = (epsilon > 0.0) ? square(epsilon / EPSILON_REF) : 0.0;
	
	// F_se
	F.se = p.F_max * f_see;


	// --- F_be --- //
	// F_be = F_max * ( (l_min - l_ce) / (l_opt * epsilon_be) ).^2
	//           -> if l_ce < l_min (0 otherwise)

	if (cur_l_ce <= p.l_min)
	{
		F.be = p.F_max * square((p.l_min - cur_l_ce) / (p.l_opt * EPSILON_BE));
	}
	else
	{
		F.be = 0.0;
	}

	// --- F_pe_star --- //
	// F_pe_star = F_max * ( (l_ce - l_opt) / (l_opt * epsilon_pe) ).^2  
	//           -> if l_ce > l_opt (0 otherwise)

	if(cur_l_ce > p.l_opt)
	{
		F.pe_star = p.F_max * square((cur_l_ce - p.l_opt) / (p.l_opt * EPSILON_PE));
	}
	else
	{
		F.pe_star = 0.0;
	}
}

/*! \brief f_l computation
 *
 * \param[in] cur_l_ce current value of 'l_ce' [m]
 */
void Muscle::fl_compute(double cur_l_ce)
{
	double this_frac;
	
	// f_l = exp(c * |(l_ce - l_opt)/(l_opt * w)|.^3)
	this_frac = fabs((cur_l_ce - p.l_opt) / (p.l_opt * W_MUSCLE));
	
	f.l = exp( C_MUSCLE * cube(this_frac) );
			
	if(f.l < F_L_INF)
	{
		f.l = F_L_INF;
	}
}

/*! \brief f_v computation
 */
void Muscle::fv_compute()
{
	// --- f_v --- //
	// f_v = (F_se + F_be) / (A * F_max * f_l + F_pe_star)

	f.v = (F.se + F.be) / (A * p.F_max * f.l + F.pe_star);
	f.v = limit_range(f.v, F_V_INF, F_V_SUP);

	// --- F_ce --- //
	// F_ce = F_se + F_be - F_pe_star * f_v;

	F.ce = F.se  + F.be - F.pe_star * f.v;

	if(F.ce < 0.0)
	{
		F.ce = 0.0;
	}

	F.m = F.se;
}

/*! \brief v_ce computation
 *
 * \param[out] cur_v_ce current derivative of 'l_ce' [m/s]
 */
void Muscle::v_ce_compute(double &cur_v_ce)
{
	// case 1 (v_ce < 0) / case 2 (v_ce >= 0)
	if (f.v < 1.0)
	{
		v.ce_norm = -p.v_max * ((1.0 - f.v) / (1.0 + K_MUSCLE * f.v));
	}
	else
	{
		v.ce_norm = -p.v_max * ((f.v - 1.0) / (7.56 * K_MUSCLE * (f.v - N_MUSCLE) + 1.0 - N_MUSCLE));
	}

	cur_v_ce = v.ce_norm * p.l_opt;
}

/*! \brief compute the derivative of 'l_ce'
 * 
 * \param[in] cur_l_ce current value of 'l_ce' [m]
 * \param[out] cur_v_ce current derivative of 'l_ce' [m/s]
 */
void Muscle::derivative(double cur_l_ce, double &cur_v_ce)
{
	forces_compute(cur_l_ce);
	fl_compute(cur_l_ce);
	fv_compute();
	v_ce_compute(cur_v_ce);
}

/*! \brief integration with Runge-Kutta method (order 4)
 * 
 * \param[in] diff_t time interval for the integration
 */
void Muscle::runge_kutta(double diff_t)
{
	double cur_l_ce;
	double K1, K2, K3, K4;

	// first step
	cur_l_ce = l.ce;
	derivative(cur_l_ce, K1);

	// second step
	cur_l_ce = pos(l.ce + 0.5 * diff_t * K1);
	derivative(cur_l_ce, K2);

	// third step
	cur_l_ce = pos(l.ce + 0.5 * diff_t * K2);
	derivative(cur_l_ce, K3);

	// fourth step
	cur_l_ce = pos(l.ce + diff_t * K3);
	derivative(cur_l_ce, K4);

	// save state variable
	v.ce = (K1 + 2.0*K2 + 2.0*K3 + K4) / 6.0;
	l.ce = pos(l.ce + diff_t * v.ce);
}

/*! \brief integration with Euler method
 * 
 * \param[in] nb_iter number of iterations [-]
 * \param[in] diff_t time interval for the integration
 */
void Muscle::euler_iterations(double diff_t, int nb_iter)
{
	double cur_l_ce, cur_v_ce, local_diff_t;

	local_diff_t = diff_t / nb_iter;

	// state initialization
	cur_l_ce = l.ce;

	for(int i=0; i<nb_iter; i++)
	{
		derivative(cur_l_ce, cur_v_ce);

		// integration
		cur_l_ce = pos(cur_l_ce + cur_v_ce * local_diff_t);
	}

	// save state variable
	v.ce = cur_v_ce;
	l.ce = cur_l_ce;
}

/*! \brief energy derivative computation
 */
double Muscle::metabolic_energy_dot()
{
	double A_dot, M_dot, S_dot, B_dot, W_dot;
	
	// Computing A_dot
	A_dot = p.m_mtu * (40*p.t_I*sin(PI_2*S) + 133*(1-p.t_I)*(1-cos(PI_2*S)));
	
	// Computing M_dot
	M_dot = p.m_mtu * g_l_ce(l.ce / p.l_opt) * (74*p.t_I*sin(PI_2*A) + 111*(1-p.t_I)*(1-cos(PI_2*A)));
	
	// Computing S_dot
	if(v.ce < 0.0)
	{
		S_dot = 0.25*F.m*(-v.ce);
	}
	else
	{
		S_dot = 0.0;
	}
	
	// Computing B_dot
	//B_dot = 1.51 * COMAN_TOTAL_MASS; // [Wang 2012]
	B_dot = 0.0225 * p.m_mtu; // [Bhargava 2004]
	
	// Computing W_dot
	if(v.ce < 0.0)
	{
		W_dot = F.ce*(-v.ce);
	}
	else
	{
		W_dot = 0.0;
	}
	
	
	// Computing E_dot
	return A_dot + M_dot + S_dot + B_dot + W_dot;
}

/*! \brief g_l_ce computation
 * 
 * \param[in] l_ce_tilde normed l_ce
 */
double Muscle::g_l_ce(double l_ce_tilde)
{
	if(l_ce_tilde <= 0.5)
	{
		return 0.5;
	}
	else if(l_ce_tilde <= 1.0)
	{
		return l_ce_tilde;
	}
	else if(l_ce_tilde <= 1.5)
	{
		return -2.0*l_ce_tilde + 3.0;
	}
	else
	{
		return 0.0;
	}
}

/*! \brief l_ce_star interpolation
 * 
 * \param[in] A activation [-]
 * \param[in] l_mtu length MTU [m]
 * \param[in] A_vec vector with the activations [-]
 * \param[in] l_mtu_vec vector with lengtha MTU [m]
 * \param[in] tab_A_l_mtu tabular with node values [m]
 * \return requested interpolation [m]
 */
double Muscle::l_ce_interp(double A, double l_mtu, double A_vec[A_INTERP_SIZE], double l_mtu_vec[L_MTU_INTERP_SIZE], double tab_A_l_mtu[A_INTERP_SIZE][L_MTU_INTERP_SIZE])
{
	int index_next_A, index_next_l_mtu;
	int ind_A_1, ind_A_2, ind_l_mtu_1, ind_l_mtu_2;

	double ab, Ab, aB, AB;
	double coef_a, coef_b;
	double a1, a2, b1, b2;


	// find correct indexes

	for(index_next_A=0; index_next_A<A_INTERP_SIZE; index_next_A++)
	{
		if (A_vec[index_next_A] >= A)
		{
			break;
		}
	}
	for(index_next_l_mtu=0; index_next_l_mtu<L_MTU_INTERP_SIZE; index_next_l_mtu++)
	{
		if (l_mtu_vec[index_next_l_mtu] >= l_mtu)
		{
			break;
		}
	}

	// bounds safety
	if (index_next_A < 1)
	{
		ind_A_1 = 0;
		ind_A_2 = 0;
	}
	else if(index_next_A >= A_INTERP_SIZE)
	{
		ind_A_1 = A_INTERP_SIZE-1;
		ind_A_2 = A_INTERP_SIZE-1;
	}
	else
	{
		ind_A_1   = index_next_A-1;
		ind_A_2   = index_next_A;
	}

	if (index_next_l_mtu < 1)
	{
		ind_l_mtu_1 = 0;
		ind_l_mtu_2 = 0;
	}
	else if(index_next_l_mtu >= L_MTU_INTERP_SIZE)
	{
		ind_l_mtu_1 = L_MTU_INTERP_SIZE-1;
		ind_l_mtu_2 = L_MTU_INTERP_SIZE-1;
	}
	else
	{
		ind_l_mtu_1   = index_next_l_mtu-1;
		ind_l_mtu_2   = index_next_l_mtu;
	}

	/*
	 * Cope with bounds
	 *
	 * ab -- Ab
	 * |     |
	 * aB -- AB
	 *
	 * a: lower A
	 * A: higher A
	 * b: lower l_mtu
	 * B: higher l_mtu
	 *  
	 */
	a1 = A_vec[ind_A_1]; // lower A
	a2 = A_vec[ind_A_2]; // higher A

	b1 = l_mtu_vec[ind_l_mtu_1]; // lower l_mtu
	b2 = l_mtu_vec[ind_l_mtu_2]; // higher l_mtu

	ab = tab_A_l_mtu[ind_A_1][ind_l_mtu_1];
	Ab = tab_A_l_mtu[ind_A_2][ind_l_mtu_1];
	aB = tab_A_l_mtu[ind_A_1][ind_l_mtu_2];
	AB = tab_A_l_mtu[ind_A_2][ind_l_mtu_2];

	// coefs: [-1 ; 1]
	if (a2 > a1)
	{
		coef_a = (2*A - a1 - a2) / (a2 - a1);
	}
	else
	{
		coef_a = 0.0;
	}

	if (b2 > b1)
	{
		coef_b = (2*l_mtu - b1 - b2) / (b2 - b1);
	}
	else
	{
		coef_b = 0.0;
	}

	return 0.25 * ( (1 + coef_a)*(1 + coef_b)*AB + (1 - coef_a)*(1 + coef_b)*aB + (1 - coef_a)*(1 - coef_b)*ab + (1 + coef_a)*(1 - coef_b)*Ab );
}

/*! \brief computes an approximation of l_ce_star with Newton-Raphson method
 * 
 * \param[in,out] l_ce_star value of l_ce (current, updated at the end) [m]
 * \param[in] cur_l_mtu current value of l_mtu [m]
 * \param[in] cur_A current value of A [-]
 */
void Muscle::l_ce_Newton_Raphson(double &l_ce_star, double cur_l_mtu, double cur_A)
{
	double F_se, F_be, F_pe_star, f_l;
	double F_se_d, F_be_d, F_pe_star_d, f_l_d;
	double F_eq, F_eq_d;

	// avoid instable equilibrium
	if (l_ce_star >= cur_l_mtu - p.l_slack)
	{
		l_ce_star = 0.999*(cur_l_mtu - p.l_slack);
	}

	if (l_ce_star < cur_l_mtu - p.l_slack)
	{
		F_se   = c.K1 *  square(cur_l_mtu - p.l_slack - l_ce_star);
		F_se_d = -2.0 * c.K1 * (cur_l_mtu - p.l_slack - l_ce_star);
	}
	else
	{
		F_se   = 0.0;
		F_se_d = 0.0;
	}

	if (l_ce_star < p.l_min)
	{
		F_be   =  c.K2 * square(p.l_min - l_ce_star);
		F_be_d = -2.0 * c.K2 * (p.l_min - l_ce_star);
	}
	else
	{
		F_be   = 0.0;
		F_be_d = 0.0;
	}

	if (l_ce_star > p.l_opt)
	{
		F_pe_star   = c.K3 * square(l_ce_star - p.l_opt);
		F_pe_star_d = 2.0 * c.K3 * (l_ce_star - p.l_opt);
	}
	else
	{
		F_pe_star   = 0.0;
		F_pe_star_d = 0.0;
	}

	if ( (p.l_opt - c.K5 <= l_ce_star) && (l_ce_star < p.l_opt) )
	{
		f_l   = exp(c.K4 * cube(p.l_opt - l_ce_star) );
		f_l_d = -3.0 * c.K4 * square(p.l_opt - l_ce_star) * f_l;
	}
	else if ( (p.l_opt <= l_ce_star) && (l_ce_star <= p.l_opt + c.K5) )
	{
		f_l   = exp(c.K4 * cube(l_ce_star - p.l_opt) );
		f_l_d = 3.0 * c.K4 * square(l_ce_star - p.l_opt) * f_l;
	}
	else
	{
		f_l   = F_L_INF;
		f_l_d = 0.0;
	}

	F_eq   = F_se   + F_be   - F_pe_star   - cur_A * p.F_max * f_l;
	F_eq_d = F_se_d + F_be_d - F_pe_star_d - cur_A * p.F_max * f_l_d;

	// -- equation solution -- //

	if (F_eq_d)
	{
		l_ce_star -= (1.0 / F_eq_d) * F_eq;
	}
}

/*! \brief iterate on 'l_ce_Newton_Raphson' to find the steady-state value of 'l_ce'
 * 
 * \param[in,out] l_ce_star value of l_ce (current, updated at the end) [m]
 * \param[in] cur_l_mtu current value of l_mtu [m]
 * \param[in] cur_A current value of A [-]
 * \return 1 if convergence achieved, 0 otherwise
 */
int Muscle::l_ce_steady_state(double &l_ce_star, double cur_l_mtu, double cur_A)
{
	double l_ce_init, l_ce_prev;

	l_ce_init = l_ce_star;
	
	// iterations
	for(int i=0; i<NB_MAX_L_CE_CONV; i++)
	{
		l_ce_prev = l_ce_star;
		l_ce_Newton_Raphson(l_ce_star, cur_l_mtu, cur_A);

		if (fabs(l_ce_star - l_ce_prev) <= THRES_L_CE_CONV)
		{
			break;
		}
	}

	// no convergence
	if (fabs(l_ce_star - l_ce_prev) > THRES_L_CE_CONV)
	{
		l_ce_star = l_ce_init;
		return 0;
	}
	else
	{
		return 1;
	}
}

/*! \brief get the minimal value of F_m, corresponding to an activation of S_MIN
 * 
 * \param[in,out] l_ce_star value of 'l_ce' for minimal force [m]
 * \param[in,out] F_min minimal value of F_m in this position [N]
 * \return 1 if convergence, 0 otherwise
 */
int Muscle::compute_F_min(double &l_ce_star, double &F_min)
{
	double epsilon_min, f_see_min;

	if (!l_ce_steady_state(l_ce_star, l.mtu, S_MIN))
	{
		return 0; // no convergence
	}

	// F_min computation
	epsilon_min = ((l.mtu - l_ce_star) - p.l_slack) / p.l_slack;
	f_see_min   = (epsilon_min > 0.0) ? square(epsilon_min / EPSILON_REF) : 0.0;

	F_min = p.F_max * f_see_min;

	return 1;
}

/*! \brief f_l computation for another l_ce in order to inverse the model
 * 
 * \param[in] l_ce l_ce current value [m]
 * \return f_l requested [-]
 */
double Muscle::fl_compute_inv(double lce)
{
	double this_frac, fl;

	// f_l = exp(c * |(l_ce - l_opt)/(l_opt * w)|.^3)
	this_frac = fabs((lce - p.l_opt) / (p.l_opt * W_MUSCLE));

	fl = exp( C_MUSCLE * cube(this_frac) );

	if(fl < F_L_INF)
	{
		fl = F_L_INF;
	}

	return fl;
}

/*! \brief compute the stimulation to get a desired force
 * 
 * \param[in] Fm_des force desired [N]
 * \return requested stimulation [-]
 */
double Muscle::inverse_compute(double Fm_des)
{
	double lse , lce, vce, Fbe, Fpe_star, fl, fv, stim, dt, denom;

	// time
	dt = inputs->get_t() - last_t_inv;
	last_t_inv = inputs->get_t();

	lse = EPSILON_REF * sqrt(Fm_des / p.F_max) * p.l_slack + p.l_slack;
	lce = l.mtu - lse;

	// finite difference order 3
	vce = (dt > 0) ? ((coef_diff_1*prev_lce[2] + coef_diff_2*prev_lce[1] + coef_diff_3*prev_lce[0] + coef_diff_4*lce ) / dt) : 0.0;

	prev_lce[2] = prev_lce[1];
	prev_lce[1] = prev_lce[0];
	prev_lce[0] = lce ;

	Fbe      = (lce <= p.l_min) ? p.F_max * square((p.l_min - lce) / (p.l_opt*EPSILON_BE)) : 0.0;
	Fpe_star = (lce  > p.l_opt) ? p.F_max * square((lce - p.l_opt) / (p.l_opt*EPSILON_PE)) : 0.0;

	fl = fl_compute_inv(lce);

	if(f.l < F_L_INF) { f.l = F_L_INF; }

	if (vce < 0)
	{
		fv = (p.v_max - vce) / (K_MUSCLE*vce + p.v_max);
	}
	else
	{
		fv = (vce*(7.56*K_MUSCLE*N_MUSCLE + N_MUSCLE -1.0) - p.v_max) / (vce*7.56*K_MUSCLE - p.v_max);
	}

	fv = limit_range(fv, F_V_INF, F_V_SUP);
	
	denom = p.F_max * fl * fv;

	// safety
	if (!denom)
	{
		return S_MIN;
	}

	stim = (Fm_des + Fbe - Fpe_star*fv) / denom;

	return limit_range(stim, S_MIN, S_MAX);
}
