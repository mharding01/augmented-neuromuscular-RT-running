import re

# return true if it is a float
def isfloat(value):
	try:
		float(value)
		return True
	except ValueError:
		return False

# return the number of parameters to optimize
def nb_opti_params(opti_file):

	nb_opti = 0

	with open(opti_file,'r') as f:

		# loop on all the lines
		for line in f:

			#detect {%f;%f} lines
			for part in re.split('{|}', line):
				elem = part.split(';')
				if len(elem) == 2:
					if isfloat(elem[0]) and isfloat(elem[1]):
						nb_opti += 1

	return nb_opti
