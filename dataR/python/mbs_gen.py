# Manipulate .mbs files to perform operations
# like combining different parts in a single one.
#
# author: Nicolas Van der Noot

import os
import xml.etree.ElementTree as ET

# set an element
def set_text(elem, value):
	elem.text = str(value)

# get the child elem (create it if non-existent)
def get_create(parent_elem, child_name):

	child_elem = parent_elem.find(child_name)

	if child_elem == None:
		ET.SubElement(parent_elem, child_name)
		child_elem = parent_elem.find(child_name)

	return child_elem

# change the name of the mbs
def change_mbsname(tree, new_name):

	mbsname = tree.getroot().find('mbsname')
	set_text(mbsname, new_name)

# remove a body from a tree
def remove_body(tree, remove_name):

	bodytree = tree.getroot().find('bodytree')

	for body in bodytree.findall('body'):
		if str(body.find('bodyname').text) == remove_name:
			bodytree.remove(body)

# add bodies from a second tree to a main tree.
# bodies are added after a first body was discovered
# to be similar between two bodies
def add_body(main_tree, second_tree):

	main_bodytree   = main_tree.getroot().find('bodytree')
	second_bodytree = second_tree.getroot().find('bodytree')

	common_found = 0

	for body_1 in main_bodytree.findall('body'):

		if common_found == 1:
			break

		for body_2 in second_bodytree.findall('body'):

			if str(body_1.find('bodyname').text) == str(body_2.find('bodyname').text):
				common_found = 1
			elif common_found == 1:
				main_bodytree.append(body_2)

# add all the bodies of the second tree to the main one
def add_all_bodies(main_tree, second_tree):

	main_bodytree   = main_tree.getroot().find('bodytree')
	second_bodytree = second_tree.getroot().find('bodytree')

	for cur_body in second_bodytree.findall('body'):
		main_bodytree.append(cur_body)

# add all the links of the second tree to the main one
def add_all_links(main_tree, second_tree):

	main_links   = get_create(main_tree.getroot(), 'links')
	second_links = second_tree.getroot().find('links')

	for cur_body in second_links.findall('link'):
		main_links.append(cur_body)

# change a joint position
def change_joint_q(tree, joint_name, new_q):

	bodytree = tree.getroot().find('bodytree')

	for cur_joint in bodytree.iter('joint'):
		if cur_joint.find('jointname').text == joint_name:

			init_value = get_create(cur_joint, 'initialvalue')
			init_q = get_create(init_value, 'q')

			set_text(init_q, new_q)

# move a joint or a point (elem) by dx, dy in the Pad
def pad_move_2D_elem(elem, dx, dy):

	if elem.find('graphics') != None:

		x2D_pos = elem.find('graphics').find('x2D').find('position')

		set_text(x2D_pos.find('x'), float(x2D_pos.find('x').text) + dx)
		set_text(x2D_pos.find('y'), float(x2D_pos.find('y').text) + dy)

# move a body by dx, dy in the Pad
def pad_move_2D_body(body, dx, dy):

	for cur_joint in body.findall('joint'):
		pad_move_2D_elem(cur_joint, dx, dy)

	for cur_point in body.findall('point'):
		pad_move_2D_elem(cur_point, dx, dy)

# move a whole tree by dx, dy in the Pad
def pad_move_2D_tree(tree, dx, dy):

	bodytree = tree.getroot().find('bodytree')

	for cur_body in bodytree.findall('body'):
		pad_move_2D_body(cur_body, dx, dy)

# print the mbs file
def print_mbs(tree, filename):

	out_temp = 'output/temp.xml'

	tree.write(out_temp)

	with open(filename, 'w') as mbs_file:
		with open(out_temp, 'r') as input_temp:	
			content = input_temp.read()
			mbs_file.seek(0,0)
			mbs_file.write('<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n')
			mbs_file.write(content)
			mbs_file.write('\r\n\r\n')

	os.remove(out_temp)


########################
##   CONFIGURATION   ###
########################

# project name
mbs_name = 'coman_spring_toe_feet'

# mbs files to load
coman_tree  = ET.parse('../elem/coman/coman_feet_missing.mbs')
ball_tree   = ET.parse('../elem/env/ball.mbs')

if mbs_name == 'coman_init_feet':
	r_foot_tree = ET.parse('../elem/feet/init/init_foot_r.mbs')
	l_foot_tree = ET.parse('../elem/feet/init/init_foot_l.mbs')
elif mbs_name == 'coman_long_feet':
	r_foot_tree = ET.parse('../elem/feet/long/long_foot_r.mbs')
	l_foot_tree = ET.parse('../elem/feet/long/long_foot_l.mbs')
elif mbs_name == 'coman_flex_feet':
	r_foot_tree = ET.parse('../elem/feet/flex/flex_foot_r.mbs')
	l_foot_tree = ET.parse('../elem/feet/flex/flex_foot_l.mbs')
elif mbs_name == 'coman_short_feet' or mbs_name == 'coman_short_feet_ball': 
	r_foot_tree = ET.parse('../elem/feet/short/short_foot_r.mbs')
	l_foot_tree = ET.parse('../elem/feet/short/short_foot_l.mbs')
elif mbs_name == 'coman_spring_toe_feet': 
	r_foot_tree = ET.parse('../elem/feet/spring_toe/spring_toe_foot_r.mbs')
	l_foot_tree = ET.parse('../elem/feet/spring_toe/spring_toe_foot_l.mbs')

# .mbs name
change_mbsname(coman_tree, mbs_name)

# add feet
add_body(coman_tree, r_foot_tree)
add_body(coman_tree, l_foot_tree)

# add ball
if mbs_name == 'coman_short_feet_ball':
	add_all_bodies(coman_tree, ball_tree)

if mbs_name == 'coman_flex_feet':
	# add links
	add_all_links(coman_tree, r_foot_tree)
	add_all_links(coman_tree, l_foot_tree)

	# modify COMAN height
	change_joint_q(coman_tree, 'FJ_T3_Coman', 0.50118)

# generate result
print_mbs(coman_tree, 'output/{}.mbs'.format(mbs_name))
