
#include "RotatingListSimu.hh"

/*! \brief constructor
 * 
 * \param[in] size size of the rotating list
 */
RotatingListSimu::RotatingListSimu(int size_list)
{
	for(int i=0; i<size_list; i++)
	{
		list.push_back(0.0);
	}

	last_index = list.size()-1;
	next_index = 0;
}

/*! \brief constructor
 * 
 * \param[in] size size of the rotating list
 * \param[in] init_val initial value (fill the whole tab with it)
 */
RotatingListSimu::RotatingListSimu(int size_list, double init_val)
{
	for(int i=0; i<size_list; i++)
	{
		list.push_back(init_val);
	}

	last_index = list.size()-1;
	next_index = 0;
}

/*! \brief destructor
 */
RotatingListSimu::~RotatingListSimu()
{

}

/*! \brief add a new value
 * 
 * \param[in] val new value to add
 */
void RotatingListSimu::add_value(double val)
{
	list[next_index] = val;

	last_index = next_index;

	next_index++;

	if (next_index >= (int) list.size())
	{
		next_index = 0;
	}
}

/*! \brief get element, counting from the last one
 * 
 * \param[in] p requested index from last
 * \return requested value
 */
double RotatingListSimu::get_from_last(int p)
{
	int request_index;

	request_index = last_index - p;

	if (request_index < 0)
	{
		request_index += list.size();
	}

	return list[request_index];
}

/*! \brief set a given value for all the elements in the rotating list
 * 
 * \param[in] val new value
 */
void RotatingListSimu::set_all_values(double val)
{
	for(unsigned int i=0; i<list.size(); i++)
	{
		list[i] = val;
	}

	last_index = list.size()-1;
	next_index = 0;
}
