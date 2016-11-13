import os
import nb_opti_params as nb_par

## BioRob cluster optimization parameters ##

# The BioRob cluster is configured by default to use the PSO optimization algorithm.
# See https://en.wikipedia.org/wiki/Particle_swarm_optimization for more details.
# The state of the optimizations can be seen using this link: http://biorobcn-gw.epfl.ch/
# To use it, you must be on the EPFL network, or use a VPN connection.

# You first need to define the number of particles that will be optimized at each generation,
# as well as the number of generations.
nb_particles   = 350 # number of particles [-]
nb_generations = 200# number of generations [-]

# Define the priority on the Biorob Cluster, which is used to handle the different
# optimizations priorities running on this cluster, at the same time.
# In normal situations, the sum of all optimizations of one person should not exceed 100.
BioRob_priority = 25 

# Set the token used on the Biorob Cluster. This one can be optained at https://biorobsrvm1.epfl.ch/token.php
# when you are connected to the EPFL network (otherwise, use a VPN connection).
# Beware: a new token must be re-generated if no optimization is performed during 3 days.
BioRob_token = "1f72626c231df7acda9b1081dbee10cb"


# generate the 'opti.xml' file
def generate_opti_xml(opti_file, opti_xml, path):

	opti_write = open(opti_xml,'w')

	opti_write.write('<?xml version="1.0" encoding="utf-8"?>\n')
	opti_write.write('<job>\n')
	opti_write.write('  <token>{}</token>\n'.format(BioRob_token))
	opti_write.write('  <priority>{}</priority>\n'.format(BioRob_priority))
	opti_write.write('  <optimizer name="pso">\n')
	opti_write.write('    <setting name="max-iterations">{}</setting>\n'.format(nb_generations))
	opti_write.write('    <setting name="population-size">{}</setting>\n\n'.format(nb_particles))
	opti_write.write('    <boundaries>\n')
	opti_write.write('    </boundaries>\n\n')
	opti_write.write('    <parameters>\n')

	for x in range(0, nb_par.nb_opti_params(opti_file)):
		opti_write.write('        <parameter name="x{}" min="0" max="1"/>\n'.format(x))

	opti_write.write('    </parameters>\n\n')
	opti_write.write('        <fitness>\n')
	opti_write.write('         <expression>(f)</expression>\n')
	opti_write.write('       </fitness>\n')
	opti_write.write('     </optimizer>\n\n')
	opti_write.write('     <dispatcher name="{}/dispatcher_opti">\n'.format(path))
	opti_write.write('     </dispatcher>\n')
	opti_write.write('   </job>\n')
	opti_write.write('   <!-- vi:ts=2:et -->\n')

	opti_write.close()

# get paths and last folder name
path = str(os.getcwd())
path_end = path.split('/')[-1]

# file names (with path)
opti_file = '{}/../../../opti/config/OptiParams.cc'.format(path)
opti_xml  = '{}/{}.xml'.format(path, path_end)

# generate xml file
generate_opti_xml(opti_file, opti_xml, path)
