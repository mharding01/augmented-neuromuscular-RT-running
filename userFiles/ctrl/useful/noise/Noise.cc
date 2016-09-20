
#include "Noise.hh"

/*! \brief constructor
 * 
 * \param[in] t current time [s]
 * \param[in] max_noise maximum noise amplitude (noise bounded in [-max_noise ; max_noise])
 * \param[in] t_period period of the noise before computing a new node [s]
 */
Noise::Noise(double t, double max_noise, double t_period)
{
	this->max_noise = max_noise;
	this->t_period  = t_period;

	last_t = t;

	last_n = get_new_noise();
	next_n = get_new_noise();
	cur_n  = last_n;
}

/*! \brief destrcutor
 */
Noise::~Noise()
{

}

/*! \brief update the noise computation
 * 
 * \param[in] t current time [s]
 */
void Noise::update(double t)
{
	double diff_t = t - last_t;

	// new random value generated
	if(diff_t >= t_period)
    {
    	cur_n  = next_n;
    	last_n = next_n;
        next_n = get_new_noise();

        last_t = t;
    }
    // interpolation between two random values
    else
    {
    	cur_n = (next_n - last_n) * (diff_t / t_period) + last_n;
    }
}
