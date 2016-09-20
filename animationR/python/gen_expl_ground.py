# example of mesh generation 

import gen_mesh as gm
import math as mt


# generate the whole example mesh (grid for rectangle)
def mesh_expl(get_z_fun):

	# bounds of the rectangular mesh
	min_x = -10.0
	max_x =  10.0

	min_y = -0.5
	max_y =  0.5

	# number of rectangles along each direction (= number of points - 1)
	nb_x = 20
	nb_y = 8

	return gm.gen_rect_mesh(min_x, max_x, min_y, max_y, nb_x, nb_y, get_z_fun)


# get the mesh height
def get_z(x, y):
	return 1.0e-2 * mt.pow(x, 2) + 1.0e1 * mt.pow(y, 3)


# generate the shape mesh
shape_mesh = mesh_expl(get_z)

# show the mesh
gm.show_mesh(shape_mesh)

# save the .stl file
shape_mesh.save('3d_file.stl')
