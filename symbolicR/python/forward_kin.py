import numpy as np
import sympy as sp
import re
import os

######################
#                    #
#    17   16   21    #
#  18     15     22  #
#  19     14     23  #
#  20     01     24  #
#      02    08      #
#     03      09     #
#     04      10     #
#     05      11     #
#     06      12     #
#     07      13     #
#                    #
######################
#
# origin: in the waist, middle point between the two pitch hip rotations
# inertial frame: located at the origin (waist), but aligned with the ground (info from IMU)
#
# Di   : position vector from the anchor point of the previous body to the current body i 
#        (previous body is not always body i-1), expressed in the relative
#        frame of the previous body
# DGi  : position vector from the anchor point of body i to its COM (center of mass) G_i,
#        expressed in the relative frame of the current body i
# Omi  : rotational vector from the previous body to the current body i 
#        (previous body is not always body i-1), expressed in the relative
#        frame of the previous body
# Rdi  : rotational matrix between body i and its predecessor
# si   : sine of the relative angle before body i
# ci   : cosine of the relative angle before body i
#
# xi   : absolute position vector (from origin, expressed in the inertial frame) 
#        of the anchor point of body i
# xgi  : absolute position vector of the COM G_i of body i
# xpi  : derivative of xi
# xgpi : derivative of xgi
# omi  : absolute rotational vector of body i
# Ri   : absolute rotational matrix
# Rti  : transpose matrix of Ri
# xji  : jacobian of 'xi'
# xgji : jacobian of 'xgi'
# Rji  : jacobian of 'Ri'

# return true if it is a float
def isInt(value):
	try:
		int(value)
		return True
	except:
		return False

# return true if it has a shape 'R%a_%b%c' (indexes %a, %b, %c also returned)
def isRot(value):
	try:
		a = int(value.split('_')[0].split('R')[1])
		b = int(value.split('_')[1][0])
		c = int(value.split('_')[1][1])
		return True, a, b, c
	except:
		return False, -1, -1, -1

# return true if it has a shape 'x%a_%b' (indexes %a, %b also returned)
def isVec(value):
	try:
		a = int(value.split('_')[0].split('x')[1])
		b = int(value.split('_')[1])
		return True, a, b
	except:
		return False, -1, -1

# count the number of 'elem' in the file
def count_elem(in_file, elem):

	count = 0;

	with open(in_file, 'r') as f:

		# loop on all the lines
		for line in f:
			cut_line = line.split(elem)
			if len(cut_line) == 2:
				count += 1

	return count

# print the declaration of an element
def print_declaration_elem(in_file, out_write, elem, nb_max_line):

	if count_elem(in_file, '{}'.format(elem)) >= 1:

		count = 0

		with open(in_file,'r') as f:

			# loop on all the lines
			for line in f:
				cut_line_1 = line.split(elem)
				cut_line_2 = line.split(' = ')

				if len(cut_line_1) == 2 and len(cut_line_2) == 2:
					if len(cut_line_2[0].split('[')) == 1:
						if count == 0:
							out_write.write('	double {}'.format(cut_line_2[0].strip()))
						else:
							out_write.write(', {}'.format(cut_line_2[0].strip()))
						count += 1

						if count >= nb_max_line:
							out_write.write(';\n')
							count = 0

		if count != 0:
			out_write.write(';\n')

# print all declarations
def print_all_declaration(in_file, out_write, nb_max_char):

	count = 0

	with open(in_file,'r') as f:

		# loop on all the lines
		for line in f:
			cut_line = line.split(' = ')

			if len(cut_line) == 2:
				if len(cut_line[0].split('[')) == 1:				
					if count == 0:
						out_write.write('	double {}'.format(cut_line[0].strip()))
					else:
						out_write.write(', {}'.format(cut_line[0].strip()))
					count += len(cut_line[0].strip()) + 2

					if count >= nb_max_char:
						out_write.write(';\n')
						count = 0

	if count != 0:
		out_write.write(';\n')

# get tilde matrix
def get_tilde(v):
	return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])

# get rotation matrix
def get_rotation_matrix(axis, direct, cosine, sine):

	if direct:
		if axis == 1:
			return np.array([[1.0, 0.0, 0.0], [0.0, cosine, sine], [0.0, -sine, cosine]])
		elif axis == 2:
			return np.array([[cosine, 0.0, -sine], [0.0, 1.0, 0.0], [sine, 0.0, cosine]])
		elif axis == 3:
			return np.array([[cosine, sine, 0.0], [-sine, cosine, 0.0], [0.0, 0.0, 1.0]])
		else:
			return np.array([])
	else:
		if axis == 1:
			return np.array([[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]])
		elif axis == 2:
			return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])
		elif axis == 3:
			return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])
		else:
			return np.array([])

# get vector axis
def get_vector_axis(axis, direct, elem):

	if direct:
		if axis == 1:
			return np.array([[elem], [0.0], [0.0]])
		elif axis == 2:
			return np.array([[0.0], [elem], [0.0]])
		elif axis == 3:
			return np.array([[0.0], [0.0], [elem]])
		else:
			return np.array([])
	else:
		if axis == 1:
			return np.array([[-elem], [0.0], [0.0]])
		elif axis == 2:
			return np.array([[0.0], [-elem], [0.0]])
		elif axis == 3:
			return np.array([[0.0], [0.0], [-elem]])
		else:
			return np.array([])

# compute the derivative of an element (for jacobian)
def der_elem(elem_str, Rj, xj, xgj, der_var):

	# element to derive (string)
	elem_str = elem_str.replace('- ','-').strip()

	# derivative axis
	der_q = int(der_var.replace('q',''))

	# detect positive/negative
	elem_split = elem_str.split('-')
	cur_len = len(elem_split)

	if cur_len == 1: # positive
		neg_flag = 0
		pos_str = elem_split[0]
	elif cur_len == 2: # negative
		neg_flag = 1
		pos_str = elem_split[1]
	else:
		print('Error: {} instead of 1 or 2 in negative detection !'.format(cur_len))
		exit()

	# compute derivative
	result = 0

	# cosine
	if pos_str == 'c{}'.format(der_q):
		result += -sp.Symbol('s{}'.format(der_q))

	# sine
	elif pos_str == 's{}'.format(der_q):
		result += sp.Symbol('c{}'.format(der_q))

	# other
	else:
		[rot_flag, a, b, c] = isRot(pos_str)
		[vec_flag, d, e] = isVec(pos_str)

		# rotation matrix
		if rot_flag:
			result += Rj[a-1][der_q-1][(b-1)*3+(c-1)]

		# vector
		elif vec_flag:
			result += xj[d-1][der_q-1][e-1]

	# apply negative
	if neg_flag:
		result = -result

	return result

# compute the derivative of an expression (for jacobian)
def symbolic_jacob_der(Rj, xj, xgj, symb_var, der_var):

	# list of all terms
	term_list = str(symb_var).replace('- ','-').replace('-','+-').split('+')

	if term_list[0] == '':
		term_list.pop(0)

	result = 0

	# loop on all terms
	for cur_term in term_list:

		# detect products
		cur_term_split = cur_term.split('*')
		cur_len = len(cur_term_split)

		# no product
		if cur_len == 1:
			result += der_elem(cur_term_split[0], Rj, xj, xgj, der_var)

		# one product
		elif cur_len == 2:
			result += der_elem(cur_term_split[0], Rj, xj, xgj, der_var)*sp.Symbol(cur_term_split[1].strip())
			result += der_elem(cur_term_split[1], Rj, xj, xgj, der_var)*sp.Symbol(cur_term_split[0].strip())
		
		# other
		else:
			print('Error: {} * counted , only implemented for 0 or 1 !'.format(cur_len-1))
			exit()

	return result

# write the beginning of the file
def write_file_beginning(out_file, joint_id_names):
	out_file.write('/*! \n')
	out_file.write(' * \\author Nicolas Van der Noot\n')
	out_file.write(' * \\file forward_kinematics.cc\n')
	out_file.write(' * \\brief forward kinematics computation for the COMAN model\n')
	out_file.write(' */\n\n')

	out_file.write('// joints enumeration\n')
	out_file.write('enum {')
	count = 0

	for i in range(1, len(joint_id_names)):

		count += 1

		if i == 1:
			out_file.write('{}'.format(get_string_enum(joint_id_names[i])))
		elif count >= 6:
			count = 0
			out_file.write(',\n	  {}'.format(get_string_enum(joint_id_names[i])))
		else:
			out_file.write(', {}'.format(get_string_enum(joint_id_names[i])))

	out_file.write('};\n\n')

	out_file.write('/*! \\brief main kinematics computation\n')
	out_file.write(' *\n')
	out_file.write(' * \\param[in,out] in_out inputs and outputs class\n')
	out_file.write(' *\n')
	out_file.write(' * computation of:\n')
	out_file.write(' *     COM (center of mass) position and velocity\n')
	out_file.write(' *     feet position, velocity and orientation\n')
	out_file.write(' *     waist and torso orientaion angles and derivatives\n')
	out_file.write(' *\n')
	out_file.write(' *   ////////////////////////\n')
	out_file.write(' *   //                    //\n')
	out_file.write(' *   //    17   16   21    //\n')
	out_file.write(' *   //  18     15     22  //\n')
	out_file.write(' *   //  19     14     23  //\n')
	out_file.write(' *   //  20     01     24  //\n')
	out_file.write(' *   //      02    08      //\n')
	out_file.write(' *   //     03      09     //\n')
	out_file.write(' *   //     04      10     //\n')
	out_file.write(' *   //     05      11     //\n')
	out_file.write(' *   //     06      12     //\n')
	out_file.write(' *   //     07      13     //\n')
	out_file.write(' *   //                    //\n')
	out_file.write(' *   ////////////////////////\n')
	out_file.write(' *\n')
	out_file.write(' * origin: in the waist, middle point between the two pitch hip rotations\n')
	out_file.write(' * inertial frame: located at the origin (waist), but aligned with the ground (info from IMU)\n')
	out_file.write(' *\n')
	out_file.write(' * Di   : position vector from the anchor point of the previous body to the current body i \n')
	out_file.write(' *        (previous body is not always body i-1), expressed in the relative\n')
	out_file.write(' *        frame of the previous body\n')
	out_file.write(' * DGi  : position vector from the anchor point of body i to its COM (center of mass) G_i,\n')
	out_file.write(' *        expressed in the relative frame of the current body i\n')
	out_file.write(' * Omi  : rotational vector from the previous body to the current body i \n')
	out_file.write(' *        (previous body is not always body i-1), expressed in the relative\n')
	out_file.write(' *        frame of the previous body\n')
	out_file.write(' * Rdi  : rotational matrix between body i and its predecessor\n')
	out_file.write(' * si   : sine of the relative angle before body i\n')
	out_file.write(' * ci   : cosine of the relative angle before body i\n')
	out_file.write(' *\n')
	out_file.write(' * xi   : absolute position vector (from origin, expressed in the inertial frame)\n')
	out_file.write(' *        of the anchor point of body i\n')
	out_file.write(' * xgi  : absolute position vector of the COM G_i of body i\n')
	out_file.write(' * xpi  : derivative of xi\n')
	out_file.write(' * xgpi : derivative of xgi\n')
	out_file.write(' * omi  : absolute rotational vector of body i\n')
	out_file.write(' * Ri   : absolute rotational matrix\n')
	out_file.write(' * Rti  : transpose matrix of Ri\n')
	out_file.write(' * xji  : jacobian of \'xi\'\n')
	out_file.write(' * xgji : jacobian of \'xgi\'\n')
	out_file.write(' * Rji  : jacobian of \'Ri\'\n')
	out_file.write(' */\n')

	out_file.write('void ForwardKinematics::main_kinematics(KinematicsInOut &in_out)\n{\n')

# compute the center of mass position and velocity
def com_compute(out_file, nb_bodies, joint_id_names, M, xg, xgp, xgj):

	out_file.write('	m_tot = ')

	for i in range(0, nb_bodies):
		out_file.write('{}'.format(M[i]))
		if i == nb_bodies-1:
			out_file.write(';\n\n')
		else:
			out_file.write(' + ')

	out_file.write('	// global com absolute position\n')
	for i in range(0, 3):
		out_file.write('	in_out.r_COM[{}] = '.format(i))
		flag_first = 0
		for j in range(0, nb_bodies):
			if flag_first:
				out_file.write(' + {}*{}'.format(M[j], xg[j][i]))
			else:
				flag_first = 1
				out_file.write('({}*xg{}_{}'.format(M[j], j+1, i+1))
			if j == nb_bodies-1:
				if flag_first:
					out_file.write(')/m_tot;\n')
				else:
					out_file.write('0.0;\n')

	out_file.write('\n')

	out_file.write('	// global com absolute velocity\n')
	for i in range(0, 3):
		out_file.write('	in_out.rp_COM[{}] = '.format(i))
		flag_first = 0
		for j in range(0, nb_bodies):
			if flag_first:
				out_file.write(' + {}*xgp{}_{}'.format(M[j], j+1, i+1))
			else:
				flag_first = 1
				out_file.write('({}*xgp{}_{}'.format(M[j], j+1, i+1))
			if j == nb_bodies-1:
				if flag_first:
					out_file.write(')/m_tot;\n')
				else:
					out_file.write('0.0;\n')
	out_file.write('\n')

	out_file.write('	// global com jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	for i in range(1, nb_bodies):
		for j in range(0, 3):
			out_file.write('		in_out.r_COM_der[{}][{}] = '.format(get_string_enum(joint_id_names[i]), j))
			flag_first = 0
			for k in range(0, nb_bodies):
				if xgj[k][i][j] != 0:
					if flag_first:
						out_file.write(' + {}*{}'.format(M[k], str(xgj[k][i][j])))
					else:
						flag_first = 1
						out_file.write('({}*{}'.format(M[k], str(xgj[k][i][j])))
				if k == nb_bodies-1:
					if flag_first:
						out_file.write(')/m_tot;\n')
					else:
						out_file.write('0.0;\n')
		if i != nb_bodies-1:
			out_file.write('\n')
		else:
			out_file.write('	}\n\n')

# from an orientation matrix, compute the roll, pitch, yaw angles (and derivative)
def yaw_pitch_roll_angles(out_file, angle_name, R_matrix, epsilon):

	if epsilon > 0: # epsilon = 1 -> pitch angle in [-pi/2 ; pi/2]
		out_file.write('	in_out.{}[0] = atan2({}, {});\n'.format(angle_name, R_matrix[5], R_matrix[8]))
		out_file.write('	in_out.{}[1] = atan2(-{}, sqrt({}*{} + {}*{}));\n'.format(angle_name, R_matrix[2], R_matrix[0], R_matrix[0], R_matrix[1], R_matrix[1]))
		out_file.write('	in_out.{}[2] = atan2({}, {});\n'.format(angle_name, R_matrix[1], R_matrix[0]))
	else: # epsilon = -1 -> pitch angle in [pi/2 ; 3*pi/2]
		out_file.write('	in_out.{}[0] = atan2(-{}, -{});\n'.format(angle_name, R_matrix[5], R_matrix[8]))
		out_file.write('	in_out.{}[1] = atan2(-{}, -sqrt({}*{} + {}*{}));\n'.format(angle_name, R_matrix[2], R_matrix[0], R_matrix[0], R_matrix[1], R_matrix[1]))
		out_file.write('	in_out.{}[2] = atan2(-{}, -{});\n'.format(angle_name, R_matrix[1], R_matrix[0]))	

# compute the time derivatives of 'yaw_pitch_roll_angles'
def theta_dot_compute(out_file, omega_in, omega_out, body_part):

	out_file.write('	in_out.{}[0] = inv_c_y_{} * (c_z_{}*{} + s_z_{}*{});\n'.format(omega_out, body_part, body_part, omega_in[0], body_part, omega_in[1]))
	out_file.write('	in_out.{}[1] = c_z_{}*{} - s_z_{}*{};\n'.format(omega_out, body_part, omega_in[1], body_part, omega_in[0]))
	out_file.write('	in_out.{}[2] = inv_c_y_{} * s_y_{} * (s_z_{}*{} + c_z_{}*{}) + {};\n'.format(omega_out, body_part, body_part, body_part, omega_in[1], body_part, omega_in[0], omega_in[2]))

# angles (position and derivative) of the waist and the torso
def torso_waist_angles(out_file, R, om, waist_id, torso_id):

	out_file.write('	// waist orientation matrix as angles [rad]\n')
	yaw_pitch_roll_angles(out_file, 'theta_waist', R[waist_id], 1)
	out_file.write('\n')

	out_file.write('	// torso orientation matrix as angles [rad]\n')
	yaw_pitch_roll_angles(out_file, 'theta_torso', R[torso_id], 1)
	out_file.write('\n')

	out_file.write('	c_y_waist = cos(in_out.theta_waist[1]);\n')
	out_file.write('	c_y_torso = cos(in_out.theta_torso[1]);\n')
	out_file.write('	c_z_waist = cos(in_out.theta_waist[2]);\n')
	out_file.write('	c_z_torso = cos(in_out.theta_torso[2]);\n\n')
	out_file.write('	s_y_waist = sin(in_out.theta_waist[1]);\n')
	out_file.write('	s_y_torso = sin(in_out.theta_torso[1]);\n')
	out_file.write('	s_z_waist = sin(in_out.theta_waist[2]);\n')
	out_file.write('	s_z_torso = sin(in_out.theta_torso[2]);\n\n')
	out_file.write('	if ((!c_y_waist) || (!c_y_torso))\n	{\n')
	out_file.write('		return;\n	}\n\n')
	out_file.write('	inv_c_y_waist = 1.0 / c_y_waist;\n')
	out_file.write('	inv_c_y_torso = 1.0 / c_y_torso;\n\n')

	out_file.write('	// waist orientation angle derivatives [rad/s]\n')
	theta_dot_compute(out_file, om[waist_id], 'omega_waist', 'waist')
	out_file.write('\n')

	out_file.write('	// torso orientation angle derivatives [rad/s]\n')
	theta_dot_compute(out_file, om[torso_id], 'omega_torso', 'torso')


# compute the feet position, velocity and orientation
def feet_compute(out_file, joint_id_names, R, x, xp, om, Rj, xj, xgj, r_foot_id, l_foot_id, x_min, x_max, y_min, y_max):

	# symbolic variables declarations
	nb_contacts = 4

	x_r_foot = x[r_foot_id]
	x_l_foot = x[l_foot_id]

	xp_r_foot = xp[r_foot_id]
	xp_l_foot = xp[l_foot_id]

	om_r_foot = om[r_foot_id]
	om_l_foot = om[l_foot_id]

	R_r_foot = R[r_foot_id]
	R_l_foot = R[l_foot_id]

	Dpt_r_foot = sp.zeros(3, 1)
	Dpt_l_foot = sp.zeros(3, 1)

	Dpt_r_foot[2] = sp.Symbol('DPT_3_16')
	Dpt_l_foot[2] = sp.Symbol('DPT_3_29')

	Dpt_r_foot_cont = nb_contacts * [None]
	Dpt_l_foot_cont = nb_contacts * [None]

	for i in range(0, nb_contacts):
		Dpt_r_foot_cont[i] = sp.zeros(3, 1)
		Dpt_l_foot_cont[i] = sp.zeros(3, 1)

	Dpt_r_foot_cont[0][0] = x_min
	Dpt_r_foot_cont[1][0] = x_min
	Dpt_r_foot_cont[2][0] = x_max
	Dpt_r_foot_cont[3][0] = x_max

	Dpt_r_foot_cont[0][1] = y_min
	Dpt_r_foot_cont[1][1] = y_max
	Dpt_r_foot_cont[2][1] = y_min
	Dpt_r_foot_cont[3][1] = y_max

	for i in range(0, nb_contacts):
		Dpt_r_foot_cont[i][2] = sp.Symbol('DPT_3_16')

	for i in range(0, nb_contacts):
		for j in range(0, 3):
			Dpt_l_foot_cont[i][j] = Dpt_r_foot_cont[i][j]

	x_r_cont = nb_contacts * [None]
	x_l_cont = nb_contacts * [None]

	# computation
	om_tilde_r_foot = get_tilde(om_r_foot)
	om_tilde_l_foot = get_tilde(om_l_foot)

	x_r = x_r_foot  + R_r_foot.T * Dpt_r_foot
	x_l = x_l_foot  + R_l_foot.T * Dpt_l_foot

	xp_r = xp_r_foot + om_tilde_r_foot * (R_r_foot.T * Dpt_r_foot)
	xp_l = xp_l_foot + om_tilde_l_foot * (R_l_foot.T * Dpt_l_foot)

	for i in range(0, nb_contacts):
		x_r_cont[i] = x_r_foot  + R_r_foot.T * Dpt_r_foot_cont[i]
		x_l_cont[i] = x_l_foot  + R_l_foot.T * Dpt_l_foot_cont[i]

	# writing outputs
	out_file.write('	// right foot absolute position\n')
	for i in range(0,3):
		out_file.write('	in_out.r_Rfoot[{}] = {};\n'.format(i, x_r[i]))
	out_file.write('\n')

	out_file.write('	// right foot absolute velocity\n')
	for i in range(0,3):
		out_file.write('	in_out.rp_Rfoot[{}] = {};\n'.format(i, xp_r[i]))
	out_file.write('\n')

	out_file.write('	// right foot jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0, 3):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, x_r[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.r_Rfoot_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// left foot absolute position\n')
	for i in range(0,3):
		out_file.write('	in_out.r_Lfoot[{}] = {};\n'.format(i, x_l[i]))
	out_file.write('\n')

	out_file.write('	// left foot absolute velocity\n')
	for i in range(0,3):
		out_file.write('	in_out.rp_Lfoot[{}] = {};\n'.format(i, xp_l[i]))
	out_file.write('\n')

	out_file.write('	// left foot jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0, 3):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, x_l[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.r_Lfoot_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// right foot contact points absolute position\n')
	for i in range(0, nb_contacts):
		for j in range(0, 3):
			out_file.write('	in_out.r_Rfoot_cont[{}][{}] = {};\n'.format(i, j, x_r_cont[i][j]))
		out_file.write('\n')

	out_file.write('	// right foot contact points jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range(0, nb_contacts):
		for j in range (1, nb_bodies):
			flag_print = 0
			for k in range(0, 3):
				cur_jac = symbolic_jacob_der(Rj, xj, xgj, x_r_cont[i][k], 'q{}'.format(j+1))
				if cur_jac != 0:
					if not flag_first:
						flag_first = 1
						flag_print = 1
					elif not flag_print:
						flag_print = 1
						out_file.write('\n')
					out_file.write('		in_out.r_Rfoot_cont_der[{}][{}][{}] = {};\n'.format(i, get_string_enum(joint_id_names[j]), k, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// left foot contact points absolute position\n')
	for i in range(0, nb_contacts):
		for j in range(0, 3):
			out_file.write('	in_out.r_Lfoot_cont[{}][{}] = {};\n'.format(i, j, x_l_cont[i][j]))
		out_file.write('\n')

	out_file.write('	// left foot contact points jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range(0, nb_contacts):
		for j in range (1, nb_bodies):
			flag_print = 0
			for k in range(0, 3):
				cur_jac = symbolic_jacob_der(Rj, xj, xgj, x_l_cont[i][k], 'q{}'.format(j+1))
				if cur_jac != 0:
					if not flag_first:
						flag_first = 1
						flag_print = 1
					elif not flag_print:
						flag_print = 1
						out_file.write('\n')
					out_file.write('		in_out.r_Lfoot_cont_der[{}][{}][{}] = {};\n'.format(i, get_string_enum(joint_id_names[j]), k, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// feet absolute orientation\n')
	for i in range(0, 9):
		out_file.write('	in_out.Rfoot_or[{}] = {};\n'.format(i, R_r_foot[i]))
	out_file.write('\n')

	for i in range(0, 9):
		out_file.write('	in_out.Lfoot_or[{}] = {};\n'.format(i, R_l_foot[i]))
	out_file.write('\n')

	out_file.write('	// right foot absolute orientation jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0,9):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, R_r_foot[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.Rfoot_or_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// left foot absolute orientation jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0,9):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, R_l_foot[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.Lfoot_or_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// right foot orientation matrix as angles [rad]\n')
	yaw_pitch_roll_angles(out_file, 'theta_Rfoot', R[r_foot_id], 1)
	out_file.write('\n')

	out_file.write('	// left foot orientation matrix as angles [rad]\n')
	yaw_pitch_roll_angles(out_file, 'theta_Lfoot', R[l_foot_id], 1)
	out_file.write('\n')

	out_file.write('	c_y_Rfoot = cos(in_out.theta_Rfoot[1]);\n')
	out_file.write('	c_y_Lfoot = cos(in_out.theta_Lfoot[1]);\n')
	out_file.write('	c_z_Rfoot = cos(in_out.theta_Rfoot[2]);\n')
	out_file.write('	c_z_Lfoot = cos(in_out.theta_Lfoot[2]);\n\n')
	out_file.write('	s_y_Rfoot = sin(in_out.theta_Rfoot[1]);\n')
	out_file.write('	s_y_Lfoot = sin(in_out.theta_Lfoot[1]);\n')
	out_file.write('	s_z_Rfoot = sin(in_out.theta_Rfoot[2]);\n')
	out_file.write('	s_z_Lfoot = sin(in_out.theta_Lfoot[2]);\n\n')
	out_file.write('	if ((!c_y_Rfoot) || (!c_y_Lfoot))\n	{\n')
	out_file.write('		return;\n	}\n\n')
	out_file.write('	inv_c_y_Rfoot = 1.0 / c_y_Rfoot;\n')
	out_file.write('	inv_c_y_Lfoot = 1.0 / c_y_Lfoot;\n\n')

	out_file.write('	// right foot orientation angle derivatives [rad/s]\n')
	theta_dot_compute(out_file, om[r_foot_id], 'omega_Rfoot', 'Rfoot')
	out_file.write('\n')

	out_file.write('	// left foot orientation angle derivatives [rad/s]\n')
	theta_dot_compute(out_file, om[l_foot_id], 'omega_Lfoot', 'Lfoot')
	out_file.write('\n')

# compute the wrists position, velocity and orientation
def wrists_compute(out_file, joint_id_names, R, x, xp, om, Rj, xj, xgj, r_elb_id, l_elb_id, r_wrist_x, r_wrist_y, r_wrist_z):

	# symbolic variables declarations
	x_r_elb = x[r_elb_id]
	x_l_elb = x[l_elb_id]

	xp_r_elb = xp[r_elb_id]
	xp_l_elb = xp[l_elb_id]

	om_r_elb = om[r_elb_id]
	om_l_elb = om[l_elb_id]

	R_r_elb = R[r_elb_id]
	R_l_elb = R[l_elb_id]

	Dpt_r_wrist = sp.zeros(3, 1)
	Dpt_l_wrist = sp.zeros(3, 1)

	Dpt_r_wrist[0] = r_wrist_x
	Dpt_r_wrist[1] = r_wrist_y
	Dpt_r_wrist[2] = r_wrist_z

	Dpt_l_wrist[0] =  r_wrist_x
	Dpt_l_wrist[1] = -r_wrist_y
	Dpt_l_wrist[2] =  r_wrist_z

	# computation
	om_tilde_r_elb = get_tilde(om_r_elb)
	om_tilde_l_elb = get_tilde(om_l_elb)

	x_r = x_r_elb  + R_r_elb.T * Dpt_r_wrist
	x_l = x_l_elb  + R_l_elb.T * Dpt_l_wrist

	xp_r = xp_r_elb + om_tilde_r_elb * (R_r_elb.T * Dpt_r_wrist)
	xp_l = xp_l_elb + om_tilde_l_elb * (R_l_elb.T * Dpt_l_wrist)

	# writing outputs
	out_file.write('	// right wrist absolute position\n')
	for i in range(0,3):
		out_file.write('	in_out.r_Rwrist[{}] = {};\n'.format(i, x_r[i]))
	out_file.write('\n')

	out_file.write('	// right wrist absolute velocity\n')
	for i in range(0,3):
		out_file.write('	in_out.rp_Rwrist[{}] = {};\n'.format(i, xp_r[i]))
	out_file.write('\n')

	out_file.write('	// right wrist jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0, 3):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, x_r[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.r_Rwrist_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// left wrist absolute position\n')
	for i in range(0,3):
		out_file.write('	in_out.r_Lwrist[{}] = {};\n'.format(i, x_l[i]))
	out_file.write('\n')

	out_file.write('	// left wrist absolute velocity\n')
	for i in range(0,3):
		out_file.write('	in_out.rp_Lwrist[{}] = {};\n'.format(i, xp_l[i]))
	out_file.write('\n')

	out_file.write('	// left wrist jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0, 3):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, x_l[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.r_Lwrist_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// wrists absolute orientation\n')
	for i in range(0, 9):
		out_file.write('	in_out.Rwrist_or[{}] = {};\n'.format(i, R_r_elb[i]))
	out_file.write('\n')

	for i in range(0, 9):
		out_file.write('	in_out.Lwrist_or[{}] = {};\n'.format(i, R_l_elb[i]))
	out_file.write('\n')

	out_file.write('	// right wrist absolute orientation jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0,9):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, R_r_elb[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.Rwrist_or_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

	out_file.write('	// left wrist absolute orientation jacobian\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range (1, nb_bodies):
		flag_print = 0
		for j in range(0,9):
			cur_jac = symbolic_jacob_der(Rj, xj, xgj, R_l_elb[j], 'q{}'.format(i+1))
			if cur_jac != 0:
				if not flag_first:
					flag_first = 1
					flag_print = 1
				elif not flag_print:
					flag_print = 1
					out_file.write('\n')
				out_file.write('		in_out.Lwrist_or_der[{}][{}] = {};\n'.format(get_string_enum(joint_id_names[i]), j, cur_jac))
	out_file.write('	}\n\n')

# get a string for the enumeration of joints
def get_string_enum(cur_string):

	cur_split = cur_string.split('_')

	if len(cur_split) >= 2:
		new_string = cur_split[0]
		for i in range(1, len(cur_split)-1):
			new_string = '{}{}'.format(new_string, cur_split[i])
	else:
		new_string = cur_string
	
	cur_split = filter(None, re.split("([A-Z][^A-Z]*)", new_string))

	new_string = cur_split[0].upper()

	for i in range(1, len(cur_split)):
		new_string = '{}_{}'.format(new_string, cur_split[i].upper())

	return new_string

# write the end of the file
def write_file_end(out_file):
	out_file.write('}\n')

# print matrix components declaration
def write_matrix_declaration(out_file, prefix):
	out_file.write('	double ')
	for i in range(0,3):
		for j in range(0,3):
			out_file.write('{}{}{}'.format(prefix, i+1, j+1))
			if i == 2 and j == 2:
				out_file.write(';\n')
			else:
				out_file.write(', ')

# print variables declaration
def write_variables_declaration(out_file, prefix, min, max):
	out_file.write('	double ')
	for i in range(min, max+1):
		out_file.write('{}{}'.format(prefix, i))
		if i == max:
			out_file.write(';\n')
		else:
			out_file.write(', ')

# variables initialization
def write_intialization(out_file, nb_bodies, joint_id_names):

	out_file.write('	// -- variables initialization -- //\n')

	out_file.write('\n	// IMU - rotation matrices\n')
	for i in range(0, 3):
		for j in range(0, 3):
			out_file.write('	IMU{}{} = in_out.IMU_Orientation[{}];\n'.format(i+1, j+1, 3*i+j))

	out_file.write('\n	// IMU - angles velocity\n')
	for i in range(0, 3):
		out_file.write('	omega_{} = in_out.IMU_Angular_Rate[{}];\n'.format(i+1, i))

	out_file.write('\n	// joint cosines\n')
	for i in range(1, nb_bodies):
		out_file.write('	c{} = cos(in_out.q_mot[{}]);\n'.format(i+1, joint_id_names[i]))

	out_file.write('\n	// joint sines\n')
	for i in range(1, nb_bodies):
		out_file.write('	s{} = sin(in_out.q_mot[{}]);\n'.format(i+1, joint_id_names[i]))

	out_file.write('\n	// joint relative velocities\n')
	for i in range(1, nb_bodies):
		out_file.write('	Om{} = in_out.qd_mot[{}];\n'.format(i+1, joint_id_names[i]))

# write symbolic vector and replace symbolic variable by its name
def write_symb_vector(out_file, vector, start_name, end_name):

	new_vector = sp.zeros(3, 1)

	flag_print = 0

	for i in range(0,3):

		if vector[i] == 0 or vector[i] == 1:
			new_vector[i] = vector[i]
		else:
			flag_print = 1

			elem_name = '{}{}{}'.format(start_name, i+1, end_name)
			out_file.write('	{} = {};\n'.format(elem_name, vector[i]).replace('1.0*',''))

			new_vector[i] = sp.Symbol(elem_name)

	if flag_print:
		out_file.write('\n')

	return new_vector

# write symbolic matrix and replace symbolic variable by its name
def write_symb_matrix(out_file, matrix, start_name, end_name):

	new_matrix = sp.zeros(3, 3)

	flag_print = 0

	for i in range(0,3):
		for j in range(0,3):

			if matrix[i,j] == 0 or matrix[i,j] == 1:
				new_matrix[i,j] = matrix[i,j]
			else:
				flag_print = 1

				elem_name = '{}{}{}{}'.format(start_name, i+1, j+1, end_name)
				out_file.write('	{} = {};\n'.format(elem_name, matrix[i,j]).replace('1.0*',''))

				new_matrix[i,j] = sp.Symbol(elem_name)

	if flag_print:
		out_file.write('\n')

	return new_matrix

# save the symbolic vector for print
def print_save_symb_vector(vector, start_name, end_name):

	new_vector  = sp.zeros(3, 1)
	save_vector = 3 * [None]

	for i in range(0,3):

		if vector[i] == 0 or vector[i] == 1:
			new_vector[i]  = vector[i]
			save_vector[i] = None
		else:
			elem_name = '{}{}{}'.format(start_name, i+1, end_name)
			save_vector[i] = '		{} = {};\n'.format(elem_name, vector[i]).replace('1.0*','')

			new_vector[i] = sp.Symbol(elem_name)

	return new_vector, save_vector

# save the symbolic matrix for print
def print_save_symb_matrix(matrix, start_name, end_name):

	new_matrix  = sp.zeros(3, 3)
	save_matrix = 9 * [None]

	for i in range(0,3):
		for j in range(0,3):

			if matrix[i,j] == 0 or matrix[i,j] == 1:
				new_matrix[i,j] = matrix[i,j]
				save_matrix[3*i+j] = None
			else:
				elem_name = '{}{}{}{}'.format(start_name, i+1, j+1, end_name)
				save_matrix[3*i+j] = '		{} = {};\n'.format(elem_name, matrix[i,j]).replace('1.0*','')

				new_matrix[i,j] = sp.Symbol(elem_name)

	return new_matrix, save_matrix

# write symbolic jacobian of a rotation matrix
def write_symb_Rj(nb_bodies, Rj, xj, xgj, Rj_print, R_matrix, index):

	# loop on all the joints
	for i in range (1, nb_bodies):

		new_matrix = sp.zeros(3, 3)

		# loop on all the matrix elements
		for j in range(0, 9):

			new_matrix[j] = symbolic_jacob_der(Rj, xj, xgj, R_matrix[j], 'q{}'.format(i+1))

		[Rj[index-1][i], Rj_print[index-1][i]] = print_save_symb_matrix(new_matrix, 'R{}_'.format(index), '_d{}'.format(i+1))

# write symbolic jacobian of an anchor point
def write_symb_xj(nb_bodies, Rj, xj, xgj, xj_print, x_vector, index):

	# loop on all the joints
	for i in range (1, nb_bodies):

		new_vector = sp.zeros(3, 1)

		# loop on all the vector elements
		for j in range(0, 3):
			new_vector[j] = symbolic_jacob_der(Rj, xj, xgj, x_vector[j], 'q{}'.format(i+1))

		[xj[index-1][i], xj_print[index-1][i]] = print_save_symb_vector(new_vector, 'x{}_'.format(index), '_d{}'.format(i+1))

# write symbolic jacobian of a com point
def write_symb_xgj(nb_bodies, Rj, xj, xgj, xgj_print, x_vector, index):

	# loop on all the joints
	for i in range (1, nb_bodies):

		new_vector = sp.zeros(3, 1)

		# loop on all the vector elements
		for j in range(0, 3):
			new_vector[j] = symbolic_jacob_der(Rj, xj, xgj, x_vector[j], 'q{}'.format(i+1))

		[xgj[index-1][i], xgj_print[index-1][i]] = print_save_symb_vector(new_vector, 'xg{}_'.format(index), '_d{}'.format(i+1))

# symbolic computation
def symbolic_computation(out_file, nb_bodies, joint_id_names, rot_axis, parent_body_index, Dpt, Dg, M):

	out_file.write('\n\n	// -- symbolic computation -- //\n')

	# Rj, xj, xgj and xgj (jacobian)
	Rj  = nb_bodies*[None]
	xj  = nb_bodies*[None]
	xgj = nb_bodies*[None]

	Rj_print  = nb_bodies*[None]
	xj_print  = nb_bodies*[None]
	xgj_print = nb_bodies*[None]
	
	for i in range(0, nb_bodies):
		Rj[i]  = nb_bodies*[None]
		xj[i]  = nb_bodies*[None]
		xgj[i] = nb_bodies*[None]

		Rj_print[i]  = nb_bodies*[None]
		xj_print[i]  = nb_bodies*[None]
		xgj_print[i] = nb_bodies*[None]

		for j in range(0, nb_bodies-1):
			Rj[i][j]  = sp.zeros(3, 3)
			xj[i][j]  = sp.zeros(3, 1)
			xgj[i][j] = sp.zeros(3, 1)

			Rj_print[i][j]  = 9 * [None]
			xj_print[i][j]  = 3 * [None]
			xgj_print[i][j] = 3 * [None]


	# rotation matrices
	out_file.write('\n	// rotation matrices\n')

	R  = nb_bodies*[None]
	Rt = nb_bodies*[None]
	Rd = nb_bodies*[None]

	Rd[0] = sp.zeros(3, 3)
	R[0]  = sp.zeros(3, 3)
	for i in range(0, 3):
		for j in range(0, 3):
			R[0][i,j] = sp.Symbol('IMU{}{}'.format(i+1, j+1))
	write_symb_Rj(nb_bodies, Rj, xj, xgj, Rj_print, R[0], 1)
	R[0]  = write_symb_matrix(out_file, R[0], 'R1_', '')
	Rt[0] = R[0].T

	for i in range(1, nb_bodies):
		Rd[i] = get_rotation_matrix(rot_axis[i], 1, sp.Symbol('c{}'.format(i+1)), sp.Symbol('s{}'.format(i+1)))
		R[i]  = Rd[i] * R[parent_body_index[i]]
		write_symb_Rj(nb_bodies, Rj, xj, xgj, Rj_print, R[i], i+1)
		R[i]  = write_symb_matrix(out_file, R[i], 'R{}_'.format(i+1), '')
		Rt[i] = R[i].T

	# jacobian rotation matrices
	out_file.write('\n	// jacobian rotation matrices\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range(0, nb_bodies):
		for j in range(1, nb_bodies):
			flag_print = 0
			for k in range(0, 9):
				if Rj_print[i][j][k] != None:
					if not flag_first:
						flag_first = 1
						flag_print = 1
					elif not flag_print:
						flag_print = 1
						out_file.write('\n')
					out_file.write('{}'.format(Rj_print[i][j][k]))
	out_file.write('	}\n')

	# omega
	out_file.write('\n	// joint absolute velocities\n')

	Om = nb_bodies*[None]
	om = nb_bodies*[None]
	om_tilde = nb_bodies*[None]

	Om[0] = sp.zeros(3, 1)
	om[0] = sp.zeros(3, 1)
	for i in range(0,3):
		om[0][i] = sp.Symbol('omega_{}'.format(i+1))
	om[0] = write_symb_vector(out_file, om[0], 'om1_', '')
	om_tilde[0] = get_tilde(om[0])

	for i in range(1, nb_bodies):
		parent_id = parent_body_index[i]
		Om[i] = get_vector_axis(rot_axis[i], 1, sp.Symbol('Om{}'.format(i+1)))
		om[i] = om[parent_id] + Rt[parent_id] * Om[i]
		om[i] = write_symb_vector(out_file, om[i], 'om{}_'.format(i+1), '')
		om_tilde[i] = get_tilde(om[i])


	# x & xp
	out_file.write('\n	// anchor point absolute positions and velocities\n')

	x  = nb_bodies*[None]
	xp = nb_bodies*[None]

	x[0]  = Rt[0] * Dpt[0]
	xp[0] = om_tilde[0] * (Rt[0] * Dpt[0])
	write_symb_xj(nb_bodies, Rj, xj, xgj, xj_print, x[0], 1)
	x[0]  = write_symb_vector(out_file, x[0], 'x1_', '')
	xp[0] = write_symb_vector(out_file, xp[0], 'xp1_', '')

	for i in range(1, nb_bodies):
		parent_id = parent_body_index[i]
		x[i]  = x[parent_id]  + Rt[parent_id] * Dpt[i]
		xp[i] = xp[parent_id] + om_tilde[parent_id] * (Rt[parent_id] * Dpt[i])
		write_symb_xj(nb_bodies, Rj, xj, xgj, xj_print, x[i], i+1)
		x[i]  = write_symb_vector(out_file, x[i], 'x{}_'.format(i+1), '')
		xp[i] = write_symb_vector(out_file, xp[i], 'xp{}_'.format(i+1), '')


	# jacobian x
	out_file.write('\n	// jacobian anchor point positions\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range(0, nb_bodies):
		for j in range(1, nb_bodies):
			flag_print = 0
			for k in range(0, 3):
				if xj_print[i][j][k] != None:
					if not flag_first:
						flag_first = 1
						flag_print = 1
					elif not flag_print:
						flag_print = 1
						out_file.write('\n')
					out_file.write('{}'.format(xj_print[i][j][k]))
	out_file.write('	}\n')


	# xg & xgp
	out_file.write('\n	// com absolute positions and velocities\n')

	xg  = nb_bodies*[None]
	xgp = nb_bodies*[None]

	for i in range(0, nb_bodies):
		xg[i] = x[i] + Rt[i] * Dg[i]
		xgp[i] = xp[i] + om_tilde[i] * (Rt[i] * Dg[i])
		write_symb_xgj(nb_bodies, Rj, xj, xgj, xgj_print, xg[i], i+1)
		xg[i]  = write_symb_vector(out_file, xg[i], 'xg{}_'.format(i+1), '')
		xgp[i] = write_symb_vector(out_file, xgp[i], 'xgp{}_'.format(i+1), '')


	# jacobian xg
	out_file.write('\n	// jacobian com absolute positions\n')
	out_file.write('	if (flag_jacob)\n	{\n')
	flag_first = 0
	for i in range(0, nb_bodies):
		for j in range(1, nb_bodies):
			flag_print = 0
			for k in range(0, 3):
				if xgj_print[i][j][k] != None:
					if not flag_first:
						flag_first = 1
						flag_print = 1
					elif not flag_print:
						flag_print = 1
						out_file.write('\n')
					out_file.write('{}'.format(xgj_print[i][j][k]))
	out_file.write('	}\n')


	# results
	out_file.write('\n	// -- Collecting results -- //\n\n')

	com_compute(out_file, nb_bodies, joint_id_names, M, xg, xgp, xgj)

	feet_compute(out_file, joint_id_names, R, x, xp, om, Rj, xj, xgj, 6, 12, -0.06, 0.08, -0.045, 0.045)

	wrists_compute(out_file, joint_id_names, R, x, xp, om, Rj, xj, xgj, 19, 23, -0.02, -0.005, -0.225)

	torso_waist_angles(out_file, R, om, 0, 15)


# generate the symbolic output file
def gen_symbolic_out(out_file_name, nb_bodies, rot_axis, parent_body_index, joint_id_names, Dpt, Dg, M):

	# temporary file
	in_temp = './{}_temp.cc'.format(out_file_name)
	file_temp = open(in_temp, 'w')

	# beginning of the file
	write_file_beginning(file_temp, joint_id_names)

	# variables initialization
	write_intialization(file_temp, nb_bodies, joint_id_names)

	# symbolic computation
	symbolic_computation(file_temp, nb_bodies, joint_id_names, rot_axis, parent_body_index, Dpt, Dg, M)	

	# end of the file
	write_file_end(file_temp)

	file_temp.close()

	# output file
	out_file = open('./{}.cc'.format(out_file_name), 'w')

	with open(in_temp, 'r') as f:

		# loop on all the lines
		for line in f:

			# declaration
			if len(line.split('// -- variables initialization -- //')) != 1:

				out_file.write('	// -- variables declaration -- //\n\n')
				print_all_declaration(in_temp, out_file, 100)
				out_file.write('\n\n')

			# copy temporary file
			out_file.write(line)

	out_file.close()

	# remove temporary file
	os.remove(in_temp)


# main script

# rotation axis for each joint before body i (1:x, 2:y, 3:z)
rot_axis = np.array([0,       # waist
			2, 1, 3, 2, 1, 2, # right leg
			2, 1, 3, 2, 1, 2, # left leg
			1, 2, 3,          # trunk
			2, 1, 3, 2,       # right arm
			2, 1, 3, 2        # left arm
			])

# parent index
parent_body_index = np.array([   -1, # waist
			0,   1,   2,  3,  4,  5, # right leg
			0,   7,   8,  9, 10, 11, # left leg
			0,  13,  14,             # trunk
			15, 16,  17,  18,        # right arm
			15, 20,  21,  22         # left arm
			])

nb_bodies = len(parent_body_index)

## anchor point positions
Dpt = nb_bodies*[None]

# waist
Dpt[0] = sp.Matrix([0.0, 0.0, 0.0])

# right leg
Dpt[1] = sp.Matrix([0.0, sp.Symbol('DPT_2_2'), 0.0])
Dpt[2] = sp.Matrix([0.0, sp.Symbol('DPT_2_6'), 0.0])
Dpt[3] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_8')])
Dpt[4] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_10')])
Dpt[5] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_12')])
Dpt[6] = sp.Matrix([0.0, 0.0, 0.0])

# left leg
Dpt[7]  = sp.Matrix([0.0, sp.Symbol('DPT_2_3'), 0.0])
Dpt[8]  = sp.Matrix([0.0, sp.Symbol('DPT_2_18'), 0.0])
Dpt[9]  = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_20')])
Dpt[10] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_22')])
Dpt[11] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_24')])
Dpt[12] = sp.Matrix([0.0, 0.0, 0.0])

# trunk
Dpt[13] = sp.Matrix([sp.Symbol('DPT_1_4'), 0.0, sp.Symbol('DPT_3_4')])
Dpt[14] = sp.Matrix([0.0, 0.0, 0.0])
Dpt[15] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_32')])

# right arm
Dpt[16] = sp.Matrix([sp.Symbol('DPT_1_36'), sp.Symbol('DPT_2_36'), sp.Symbol('DPT_3_36')])
Dpt[17] = sp.Matrix([0.0, sp.Symbol('DPT_2_39'), 0.0])
Dpt[18] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_41')])
Dpt[19] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_43')])

# left arm
Dpt[20] = sp.Matrix([sp.Symbol('DPT_1_37'), sp.Symbol('DPT_2_37'), sp.Symbol('DPT_3_37')])
Dpt[21] = sp.Matrix([0.0, sp.Symbol('DPT_2_46'), 0.0])
Dpt[22] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_48')])
Dpt[23] = sp.Matrix([0.0, 0.0, sp.Symbol('DPT_3_50')])

## COM positions
Dg = nb_bodies*[None]

# waist
Dg[0] = sp.Matrix([sp.Symbol('L_1_6'), sp.Symbol('L_2_6'), sp.Symbol('L_3_6')])

# right leg
Dg[1] = sp.Matrix([sp.Symbol('L_1_7') , sp.Symbol('L_2_7') , sp.Symbol('L_3_7')])
Dg[2] = sp.Matrix([sp.Symbol('L_1_8') , sp.Symbol('L_2_8') , sp.Symbol('L_3_8')])
Dg[3] = sp.Matrix([sp.Symbol('L_1_9') , sp.Symbol('L_2_9') , sp.Symbol('L_3_9')])
Dg[4] = sp.Matrix([sp.Symbol('L_1_10'), sp.Symbol('L_2_10'), sp.Symbol('L_3_10')])
Dg[5] = sp.Matrix([sp.Symbol('L_1_11'), sp.Symbol('L_2_11'), sp.Symbol('L_3_11')])
Dg[6] = sp.Matrix([sp.Symbol('L_1_12'), 0.0                , sp.Symbol('L_3_12')])

# left leg
Dg[7]  = sp.Matrix([sp.Symbol('L_1_13'), sp.Symbol('L_2_13'), sp.Symbol('L_3_13')])
Dg[8]  = sp.Matrix([sp.Symbol('L_1_14'), sp.Symbol('L_2_14'), sp.Symbol('L_3_14')])
Dg[9]  = sp.Matrix([sp.Symbol('L_1_15'), sp.Symbol('L_2_15'), sp.Symbol('L_3_15')])
Dg[10] = sp.Matrix([sp.Symbol('L_1_16'), sp.Symbol('L_2_16'), sp.Symbol('L_3_16')])
Dg[11] = sp.Matrix([sp.Symbol('L_1_17'), sp.Symbol('L_2_17'), sp.Symbol('L_3_17')])
Dg[12] = sp.Matrix([sp.Symbol('L_1_18'), 0.0                , sp.Symbol('L_3_18')])

# trunk
Dg[13] = sp.Matrix([sp.Symbol('L_1_19'), sp.Symbol('L_2_19'), sp.Symbol('L_3_19')])
Dg[14] = sp.Matrix([sp.Symbol('L_1_20'), sp.Symbol('L_2_20'), sp.Symbol('L_3_20')])
Dg[15] = sp.Matrix([sp.Symbol('L_1_21'), sp.Symbol('L_2_21'), sp.Symbol('L_3_21')])

# right arm
Dg[16] = sp.Matrix([sp.Symbol('L_1_22'), sp.Symbol('L_2_22'), sp.Symbol('L_3_22')])
Dg[17] = sp.Matrix([sp.Symbol('L_1_23'), sp.Symbol('L_2_23'), sp.Symbol('L_3_23')])
Dg[18] = sp.Matrix([sp.Symbol('L_1_24'), sp.Symbol('L_2_24'), sp.Symbol('L_3_24')])
Dg[19] = sp.Matrix([sp.Symbol('L_1_25'), sp.Symbol('L_2_25'), sp.Symbol('L_3_25')])

# left arm
Dg[20] = sp.Matrix([sp.Symbol('L_1_26'), sp.Symbol('L_2_26'), sp.Symbol('L_3_26')])
Dg[21] = sp.Matrix([sp.Symbol('L_1_27'), sp.Symbol('L_2_27'), sp.Symbol('L_3_27')])
Dg[22] = sp.Matrix([sp.Symbol('L_1_28'), sp.Symbol('L_2_28'), sp.Symbol('L_3_28')])
Dg[23] = sp.Matrix([sp.Symbol('L_1_29'), sp.Symbol('L_2_29'), sp.Symbol('L_3_29')])

# masses
M = np.array([ 'M_6', # waist
			'M_7' , 'M_8' , 'M_9' , 'M_10', 'M_11', 'M_12', # right leg
			'M_13', 'M_14', 'M_15', 'M_16', 'M_17', 'M_18', # left leg
			'M_19', 'M_20', 'M_21',   # trunk
			'M_22', 'M_23', 'M_24', 'M_25', # right arm
			'M_26', 'M_27', 'M_28', 'M_29'  # left arm
			])

# joint names
joint_id_names = np.array(['0', # waist
			'RightHipPitch_id', 'RightHipRoll_id', 'RightHipYaw_id', 'RightKneePitch_id', 'RightFootRoll_id', 'RightFootPitch_id', # right leg
			'LeftHipPitch_id' , 'LeftHipRoll_id' , 'LeftHipYaw_id' , 'LeftKneePitch_id' , 'LeftFootRoll_id' , 'LeftFootPitch_id' , # left leg
			'TorsoRoll_id'    , 'TorsoPitch_id'  , 'TorsoYaw_id'   , # trunk
			'RightShPitch_id' , 'RightShRoll_id' , 'RightShYaw_id' , 'RightElbPitch_id', # right arm
			'LeftShPitch_id'  , 'LeftShRoll_id'  , 'LeftShYaw_id'  , 'LeftElbPitch_id'   # left arm
			])

out_file_name = 'forward_kinematics'

gen_symbolic_out(out_file_name, nb_bodies, rot_axis, parent_body_index, joint_id_names, Dpt, Dg, M)
