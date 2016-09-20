# Generate a .stl file corresponding to a 3D surface.
# This can be for instance used to design visualization files for
# the GCM (ground contact model) or for the robot feet.
# It is then possible to import it in Blender (https://www.blender.org/)
# to extrude it, to add colors, and then to generate VRML (.wrl) files.
# This VRML files can be used in the Robotran (http://www.robotran.be/) visualizer.
#
# This file uses stl-np: https://github.com/WoLpH/numpy-stl

import numpy as np
from stl.mesh import Mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

# data of the mesh
class DataMesh:

	# constructor
	def __init__(self, nb_vec):
		self.iter   = 0
		self.nb_vec = nb_vec
		self.data   = np.zeros(nb_vec, dtype=Mesh.dtype)

	# add new mesh triangle
	def add_mesh_tr(self, new_vec):
		if self.iter >= self.nb_vec:
			print 'Error: too many ({}) vectors added (max: {}) !'.format(self.iter+1, self.nb_vec)
		else:
			self.data['vectors'][self.iter] = new_vec
		self.iter += 1

	# get the final mesh
	def get_mesh(self):
		the_mesh = Mesh(self.data, remove_empty_areas=False)
		the_mesh.update_units()
		return the_mesh


# show the resulting mesh
def show_mesh(the_mesh):

	# figure creation
	figure = pyplot.figure()
	axes   = mplot3d.Axes3D(figure)

	vec = the_mesh.vectors
	vec_len = len(vec)

	collection = mplot3d.art3d.Poly3DCollection(vec, facecolors='b')
	axes.add_collection3d(collection)

	min_x = vec[0][0][0]
	max_x = vec[0][0][0]
	min_y = vec[0][0][1]
	max_y = vec[0][0][1]
	min_z = vec[0][0][2]
	max_z = vec[0][0][2]

	for i in range(0, vec_len):
		for j in range(0, 3):

			cur_x = vec[i][j][0]
			cur_y = vec[i][j][1]
			cur_z = vec[i][j][2]

			if cur_x < min_x:
				min_x = cur_x
			elif cur_x > max_x:
				max_x = cur_x

			if cur_y < min_y:
				min_y = cur_y
			elif cur_y > max_y:
				max_y = cur_y

			if cur_z < min_z:
				min_z = cur_z
			elif cur_z > max_z:
				max_z = cur_z

	axes.set_xlabel('X')
	axes.set_ylabel('Y')
	axes.set_zlabel('Z')

	axes.set_xlim3d(min_x, max_x)
	axes.set_ylim3d(min_y, max_y)
	axes.set_zlim3d(min_z, max_z)

	# show the plot to the screen
	pyplot.show()


# generate triangle to be used by the mesh
def gen_triangle(x0, y0, x1, y1, x2, y2, get_z_fun):

	return np.array([ 
		[x0, y0, get_z_fun(x0, y0)], 
		[x1, y1, get_z_fun(x1, y1)], 
		[x2, y2, get_z_fun(x2, y2)] 
		])


# generate the whole mesh for a rectangular shape
def gen_rect_mesh(min_x, max_x, min_y, max_y, nb_x, nb_y, get_z_fun):
		
	# increment of distance along each direction
	delta_x = (max_x - min_x) / nb_x
	delta_y = (max_y - min_y) / nb_y

	data_mesh = DataMesh(2*nb_x*nb_y)

	# add vector
	for i in range(0, nb_x):
		for j in range(0, nb_y):

			cur_x_min = min_x + i*delta_x
			cur_y_min = min_y + j*delta_y

			cur_x_max = min_x + (i+1)*delta_x
			cur_y_max = min_y + (j+1)*delta_y

			data_mesh.add_mesh_tr(gen_triangle(cur_x_min, cur_y_min, cur_x_max, cur_y_min, cur_x_min, cur_y_max, get_z_fun))
			data_mesh.add_mesh_tr(gen_triangle(cur_x_max, cur_y_min, cur_x_max, cur_y_max, cur_x_min, cur_y_max, get_z_fun))

	return data_mesh.get_mesh()
