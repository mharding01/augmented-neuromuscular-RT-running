import re
import os
import nb_opti_params as nb_par


# return true if it is a float
def isfloat(value):
	try:
		float(value)
		return True
	except ValueError:
		return False


# generate 'OptiGeneration.cc'
def generate_OptiGeneration(opti_file, opti_gen):

	count = 0

	opti_write = open(opti_gen,'w')

	with open(opti_file,'r') as f:

		# loop on all the lines
		for line in f:

			# lines replacements for OptiGeneration
			line = line.replace('OPTI_NAME','OptiGeneration')

			#detect {%f;%f} lines
			for part in re.split('{|}', line):
				elem = part.split(';')
				if len(elem) == 2:
					if isfloat(elem[0]) and isfloat(elem[1]):
						line = line.replace('{{{}}}'.format(part),'optiParams[{}]'.format(count))
						count += 1

			# write line
			opti_write.write(line)

	opti_write.close()


# generate 'get_optiParams.cc'
def generate_get_optiParams(opti_file, opti_param):

	count = 0

	opti_write = open(opti_param,'w')

	opti_write.write('#include "opti_params.hh"\n')
	opti_write.write('#include <iostream>\n')
	opti_write.write('#include <cstdlib>\n\n')
	opti_write.write('/*! \\brief get the optimized parameters exact values\n')
	opti_write.write(' * \n')
	opti_write.write(' * \param[in] optiNorms normalized optimization parameters\n')
	opti_write.write(' * \\return real parameter corresponding to the last normalized parameter added\n')
	opti_write.write(' */\n')
	opti_write.write('double convert_to_optiParams(std::vector<double> optiNorms)\n')
	opti_write.write('{\n')
	opti_write.write('	int index;\n')
	opti_write.write('	double last_elem;\n\n')
	opti_write.write('	index = optiNorms.size() - 1;\n\n')
	opti_write.write('	if (index < 0)\n')
	opti_write.write('	{\n')
	opti_write.write('		std::cout << "Error: optiNorms is empty !" << std::endl;\n')
	opti_write.write('		exit(EXIT_FAILURE);\n')
	opti_write.write('	}\n\n')
	opti_write.write('	last_elem = optiNorms[index];\n\n')
	opti_write.write('	switch (index)\n')
	opti_write.write('	{\n')

	with open(opti_file,'r') as f:

		# loop on all the lines
		for line in f:

			#detect {%f;%f} lines
			for part in re.split('{|}', line):
				elem = part.split(';')
				if len(elem) == 2:
					if isfloat(elem[0]) and isfloat(elem[1]):

						min_val  = float(elem[0])
						max_val  = float(elem[1])
						diff_val = max_val - min_val

						opti_write.write('		case {} : return (last_elem * {:.8f} + {:.8f});\n'.format(count, diff_val, min_val))

						count += 1

	opti_write.write('\n		default:\n')
	opti_write.write('			std::cout << "Error: optiNorms is too small: " << index << " !" << std::endl;\n')
	opti_write.write('			exit(EXIT_FAILURE);\n')
	opti_write.write('			break;\n')
	opti_write.write('	}\n')
	opti_write.write('}\n\n')
	opti_write.write('/*! \\brief get the number of optimized parameters\n')
	opti_write.write(' * \n')
	opti_write.write(' * \\return number of optimized parameters\n')
	opti_write.write(' */\n')
	opti_write.write('int get_nb_optiParams()\n')
	opti_write.write('{\n')
	opti_write.write('	return {};\n'.format(nb_par.nb_opti_params(opti_file)))
	opti_write.write('}\n')

	opti_write.close()


# get paths
path = str(os.getcwd())
rel_path_gen = '../../userFiles/simu/io/opti/generated'

# file names (with path)
opti_file  = '{}/../config/OptiParams.cc'.format(path)
opti_gen   = '{}/{}/OptiGeneration.cc'.format(path, rel_path_gen)
opti_param = '{}/{}/get_optiParams.cc'.format(path, rel_path_gen)

# generate optimization preparation files
generate_OptiGeneration(opti_file, opti_gen)
generate_get_optiParams(opti_file, opti_param)

print '{} generated'.format(opti_gen)
print '{} generated'.format(opti_param)
