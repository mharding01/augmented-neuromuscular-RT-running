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
def generate_OptiResults(file_db, opti_file, opti_norm, opti_res):

	# extract indexes corresponding to the best fitness
	command = "sqlite3 {} 'select `iteration`,`index`,fitness.value parameter_values FROM fitness ORDER by value DESC limit 0,1;'".format(file_db)
	best_run = os.popen(command).read()
	best_run = best_run.rstrip('\n').split('|')

	iteration = best_run[0] # generation
	index     = best_run[1] # index of the particle
	fitness   = best_run[2] # fitness value

	command = "sqlite3 {} 'select * from parameter_values where `iteration`={} and `index`={};'".format(file_db, iteration, index)

	params = os.popen(command).read()

	params = params.rstrip('\n').split('|')

	# remove two first elements
	for i in range(0,2):
		del params[0]

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
path_end = path.split('/')[-1]

# input file names (with path)
file_db   = '{}/{}.1.db'.format(path, path_end)
opti_file = '{}/../../../opti/config/OptiParams.cc'.format(path)

# output file names (with path)
opti_norm = '{}/results/out_norm.txt'.format(path)
opti_res  = '{}/results/OptiResults.cc'.format(path)

# generate result file
generate_OptiResults(file_db, opti_file, opti_norm, opti_res)

