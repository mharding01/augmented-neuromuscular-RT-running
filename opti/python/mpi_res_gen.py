import re
import os

# return true if it is a float
def isfloat(value):
	try:
		float(value)
		return True
	except ValueError:
		return False


# generate 'out_norm.txt' and 'OptiResults.cc'
def generate_OptiResults(file_in, opti_file, opti_norm, opti_res):

	# get fitness and best params
	params = []
	start_params = 0

	# open input results files
	with open(file_in,'r') as f:

		# loop on all the lines
		for line in f:

			# remove '\n'
			line = line.rstrip()

			split_1 = line.split(': ')

			if len(split_1) == 2:
				if split_1[0] == 'best fitness':
					fitness = split_1[1]

			split_2 = line.split(':')

			if len(split_2) == 2:
				if split_2[0] == 'best particle':
					start_params = 1

			if (start_params == 1) and isfloat(line):
				params.append(float(line))


	# print output norms
	out_norm = open(opti_norm,'w')

	out_norm.write('fitness: {}\n\n'.format(fitness))
	out_norm.write('norms:\n')

	for i in range(0, len(params)):
		out_norm.write('{}\n'.format(params[i]))

	out_norm.write('\n\n')
	out_norm.write('/*\n * The following lines present the results file with the normalized parameters.\n')
	out_norm.write(' * Do not use it directly, its purpose is just to show the match between the parameters\n')
	out_norm.write(' * to optimize and their normalized optimized values.\n */\n\n')

	# use opti_file informations
	count = 0

	out_params = open(opti_res,'w')

	with open(opti_file,'r') as f:

		# loop on all the lines
		for line in f:

			# lines replacements for OptiResults
			line_n = line.replace('OPTI_NAME','OptiNorms')
			line   = line.replace('OPTI_NAME','OptiResults')

			#detect {%f;%f} lines
			for part in re.split('{|}', line):
				elem = part.split(';')
				if len(elem) == 2:
					if isfloat(elem[0]) and isfloat(elem[1]):

						# compute optimized value
						norm_par = float(params[count])
						min_val  = float(elem[0])
						max_val  = float(elem[1])
						diff_val = max_val - min_val
						
						val = min_val + norm_par * diff_val

						# modify line
						line   = line.replace('{{{}}}'.format(part),'{:.8f}'.format(val))
						line_n = line_n.replace('{{{}}}'.format(part),'{:.8f}'.format(norm_par))
						count += 1

			# write line
			out_params.write(line)
			out_norm.write(line_n)

	# close files
	out_norm.close()
	out_params.close()


# get paths and last folder name
path = str(os.getcwd())

# input file names (with path)
file_in   = '{}/opti_res.txt'.format(path)
opti_file = '{}/../../../opti/config/OptiParams.cc'.format(path)

# output file names (with path)
opti_norm = '{}/results/out_norm.txt'.format(path)
opti_res  = '{}/results/OptiResults.cc'.format(path)

# generate result file
generate_OptiResults(file_in, opti_file, opti_norm, opti_res)
