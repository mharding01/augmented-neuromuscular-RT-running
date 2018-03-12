# This file analyses the speed tracking of COMAN when
# a speed reference is provided (and modulated).

# import libraries
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

# compute the running average (period = t_av) of a vector x, evolving with time t
def running_average(t, x, t_av):

	# vector length
	vec_len = len(t)

	# safety
	if vec_len != len(x):
		print("error: vector sizes do not match !")
		return

	# total time
	diff_t = t[-1] - t[0]

	# semi length of the running average
	av_semi_len = int(0.5 * (t_av / diff_t) * vec_len)

	# total length of the running average
	av_len = 2 * av_semi_len + 1

	# cumulative sum
	cum_sum = np.cumsum(x, dtype=float)

	# output vector
	y = np.zeros((vec_len))

	# loop on all elements, except borders
	for i in range(av_semi_len, vec_len - av_semi_len):
		y[i] = float(cum_sum[i+av_semi_len] - cum_sum[i-av_semi_len] + x[i-av_semi_len]) / float(av_len)

	# initial border
	for i in range(0, av_semi_len):
		y[i] = float(cum_sum[2*i]) / float(2*i+1)

	# final border
	for i in range(vec_len - av_semi_len, vec_len):
		y[i] = float(cum_sum[vec_len-1] - cum_sum[vec_len-av_len] + x[vec_len-av_len]) / float(av_len)

	# return result
	return y

# linear interpolation between (t_0, y_0) and (t_end, y_end) at time 't'
def linear_interpol(y_0, y_end, t_0, t_end, t):

	if (t < t_0) :
		return y_0
	elif (t < t_end) :
		return y_0 + (t - t_0) / (t_end - t_0) * (y_end - y_0)
	else :
		return y_end

# get data
data_real = np.loadtxt('real_speed.res')
data_ref  = np.loadtxt('target_speed.res')

# time
t = data_real[:,0]
one_sec_nb_meas = int(len(t) / (t[-1] - t[0]))

# real speed
real_sp = data_real[:,1]
real_av = running_average(t, real_sp, 1.0)

# target speed
targ_sp = data_ref[:,1]

# movie properties
movie_fqc  = 60
movie_time = 74*16 # for slow down with a factor 16

# output folder
output_folder = './'

# 1 to get the animation, 0 otherwise
anim = 0

# 1 to save the figure, 0 otherwise
save_fig = 0

# initialization of the animation
def init_movie():
	for line in lines:
		line.set_data([],[])
	return lines

# animation function, called sequentially
def animate_movie(i):

	# get correct time index
	ind = (i * one_sec_nb_meas / movie_fqc) - 5 * one_sec_nb_meas
	
	# slow down by a factor 16
	ind = int(ind / 16)

	# plot not started
	if ind < 0:
		return lines
	
	# get vectors
	x = t[0:ind]

	y1 = targ_sp[0:ind]
	y2 = real_av[0:ind]

	# fill lines
	for lnum,line in enumerate(lines):
		if lnum == 0:
			line.set_data(x, y1)
		elif lnum == 1:
			line.set_data(x, y2)

	return lines

# static figure
if not anim:
	fig = plt.figure(figsize=(10,5))
	ax = plt.gca()

	plt.plot(t, targ_sp, 'b',linestyle='--', dashes=(12, 3), lw=3.5, label='target speed')
	plt.plot(t, real_av, 'r', lw=3.5, label='actual speed')
	plt.xlabel("time (s)", fontsize=15, fontweight='bold')
	plt.ylabel("speed (m/s)", fontsize=15, fontweight='bold')
	plt.xlim([0, 70.0])
	plt.ylim([1.1, 1.9])

	# space for labels
	ax.xaxis.labelpad = 10
	ax.yaxis.labelpad = 15

	# hide right and top spines
	ax.spines['right'].set_visible(False)
	ax.spines['top'].set_visible(False)

	# spines width
	ax.spines['left'].set_linewidth(2.0)
	ax.spines['bottom'].set_linewidth(2.0)

	# only show ticks on left and bottom spines
	ax.yaxis.set_ticks_position('left')
	ax.xaxis.set_ticks_position('bottom')

	# ticks parameters
	plt.tick_params(axis='both', which='major', labelsize=15, direction='out', width=2)

	# legend
	plt.legend(loc='upper right', frameon=False, fontsize=20)

#	# screenshots time
#	plt.plot([12.35, 12.90], [1.37, 1.37], 'black', lw=3)
#	plt.plot([12.35, 12.35], [1.365, 1.375], 'black', lw=3)
#	plt.plot([12.90, 12.90], [1.365, 1.375], 'black', lw=3)
#
#	ax.text((12.35+12.90)/2.0, 1.41, 'snapshots', horizontalalignment='center', verticalalignment='center', fontsize=13, color='black')

	if save_fig:
		plt.savefig('{}/sp_track.eps'.format(output_folder), bbox_inches='tight', pad_inches=0.8)

# animation
else:

	# set up figure and axes
	fig = plt.figure(figsize=(30, 18))
	extra_border = 14
	ax = plt.axes(xlim=(0-extra_border, 70+extra_border), ylim=(1.0, 2.0))
	ax.arrow(0.0, 1.1, 72.5,  0.0, head_width=0.025, head_length=1.5, fc='#000033', ec='#000033', linewidth=5)
	ax.arrow(0.0, 1.1,  0.0, 0.75, head_width=0.8, head_length=0.035, fc='#000033', ec='#000033', linewidth=5)

	# horizontal axis
	for i in range(1,8):
		plt.plot([i*10, i*10], [1.1-0.015, 1.1], color='#000033', lw=5)
		ax.text(i*10, 1.05, i*10, weight='bold', horizontalalignment='center', verticalalignment='center', fontsize=30, color='#000033')

	ax.text(80, 1.0, 'time (s)', weight='bold', horizontalalignment='center', verticalalignment='center', fontsize=40, color='#000033')

	# vertical axis
	for i in range(1,8):
		plt.plot([-0.5, 0.0], [1.1+i*0.1, 1.1+i*0.1], color='#000033', lw=5)
		ax.text(-1.0, 1.1+i*0.1, 1.1+i*0.1, weight='bold', horizontalalignment='right', verticalalignment='center', fontsize=30, color='#000033')

	ax.text(-2.5, 1.95, 'speed (m/s)', weight='bold', horizontalalignment='center', verticalalignment='center', fontsize=40, color='#000033')

	# legend
	plt.plot([55, 59], [1.9, 1.9], 'b', lw=8)
	plt.plot([55, 59], [1.8, 1.8], 'r', lw=8)
	ax.text(60, 1.9, 'target speed', weight='bold', horizontalalignment='left', verticalalignment='center', fontsize=40, color='#000033')
	ax.text(60, 1.8, 'actual speed', weight='bold', horizontalalignment='left', verticalalignment='center', fontsize=40, color='#000033')

	# set up lines
	lines = []
	color_tab = ['b', 'r']
	for i in range(2):
		lobj = ax.plot([],[], lw=8, color=color_tab[i])[0]
		lines.append(lobj)

	# call the animator
	anim = animation.FuncAnimation(fig, animate_movie, init_func=init_movie,
		frames=movie_fqc*movie_time, interval=1000/movie_fqc, blit=True)

	anim.save('sp_track.mp4', fps=movie_fqc, extra_args=['-vcodec', 'libx264'])

plt.show()
