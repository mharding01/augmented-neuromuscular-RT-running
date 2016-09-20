import re
import os
from itertools import izip

# return true if it is a float
def isfloat(value):
	try:
		float(value)
		return True
	except ValueError:
		return False

# generate 'opti_norm.txt' from 'OptiResults.cc' and 'OptiParams.CC'
def generate_ResultsNorm(opti_param, opti_res, opti_norm):

	out_norm = open(opti_norm,'w')
	count = 0

	out_norm.write('#include "opti_params.hh"\n')
	out_norm.write('#include <iostream>\n')
	out_norm.write('#include <cstdlib>\n\n')
	out_norm.write('/*! \\brief extact normed results from OptiResults.cc and OptiParams.cc\n')
	out_norm.write(' * \n')
	out_norm.write(' * \param[in] index inde\n')
	out_norm.write(' * \\return normed result\n')
	out_norm.write(' */\n')
	out_norm.write('double new_norm_start(int index)\n')
	out_norm.write('{\n')
	out_norm.write('	switch (index)\n')
	out_norm.write('  	{\n')

	with open(opti_param,'r') as f1, open(opti_res,'r') as f2: 
		for line1, line2 in izip(f1, f2):
			for part in re.split('{|}', line1):
				elem = part.split(';')
				if len(elem) == 2:
					if isfloat(elem[0]) and isfloat(elem[1]):
						min_val  = float(elem[0])
						max_val  = float(elem[1])
						diff_val = max_val - min_val
						if diff_val == 0.0:
							diff_val = 1.0

			for elem in re.findall('-?\d+\.\d+', line2):
				result = float(elem)
				xstart = (result-min_val)/diff_val
				if xstart > 1.0:
					xstart = 1.0
				if xstart < 0.0:
					xstart = 0.0
				out_norm.write('		case {} : return {:.8f};\n'.format(count,xstart))
				count += 1

	out_norm.write('\n		default:\n')
	out_norm.write('			std::cout << "Error: OptiResults is too small: " << index << " !" << std::endl;\n')
	out_norm.write('			exit(EXIT_FAILURE);\n')
	out_norm.write('			break;\n')
	out_norm.write('	}\n')
	out_norm.write('}\n\n')

	# close files
	out_norm.close()


# get paths and last folder name
path = str(os.getcwd())

# input file names (with path)
opti_res  = '{}/../../userFiles/ctrl/io/opti/specific_opti/OptiResults.cc'.format(path)
opti_param = '{}/../config/OptiParams.cc'.format(path)

# output file names (with path)
opti_norm = '{}/../../userFiles/simu/io/opti/generated/StartNormValues.cc'.format(path)

# generate result file
generate_ResultsNorm(opti_param, opti_res, opti_norm)
