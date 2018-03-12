# This file is used by both "steer_params.py" and "straight_params.py".
# The class implemented in this file provides fields to stock the data
# and different methods to analyze them.

# import libraries
import scipy
from scipy.stats import f
import numpy as np
from matplotlib import pyplot as plt
import math as mt

# return true if it is a float
def isFloat(value):
	try:
		float(value)
		return True
	except:
		return False

# gather all data
class AllData():
	"""gather all the data"""

	def __init__(self, names, plot_fac, labels, approx_order, bounds, x_star, p_thres, plot_xlim, plot_ylim, output_folder):

		# safety
		if len(bounds.shape) != 2:
			print('Error: bounds must be a 2D tabular !')
			exit(1)

		if len(approx_order) != len(names):
			print('Error: approx_order size ({}) != names size({})'.format(len(approx_order), len(names)))
			exit(1)

		if bounds.shape[0] != len(names):
			print('Error: bounds size ({}) != names size({}) !'.format(bounds.shape[0], len(names)))
			exit(1)

		self.names  = names  # names of the parameters
		self.plot_fac = plot_fac # plot factors for unit comversion
		self.labels = labels # labels for the plots
		self.bounds = bounds # optimization bounds
		self.x_star = x_star # reference speed
		self.approx_order = approx_order # approximation order
		self.nb_params  = len(names) # number of parameters (outputs)
		self.nb_tot     = 0 # total number of measures
		self.nb_sets    = 0 # number of sets (number of speed references)
		self.nb_subplot = 0 # number of sub-plots
		self.p_thres    = p_thres # threshold for p-value
		self.plot_xlim  = plot_xlim # xlim for plots
		self.plot_ylim  = plot_ylim # xlim for plots
		self.output_folder = output_folder # output folder to save the graphs

		self.data       = [] # gather results data
		self.out        = [] # 'self.data' without scaling
		self.nb_trials  = [] # number of trials for each set
		self.speed_ref  = [] # speed reference for each set
		self.speed_av   = [] # average of the real speeds (for each set)
		self.out_av     = [] # average of each data (for each set)
		self.speed_std  = [] # standard deviations of the real speeds (for each set)
		self.out_std    = [] # standard deviations of each data (for each set)
		self.speed_trial_av = [] # average of the speed of each trial in each set

		self.flag_save = 0 # 1 to save the result as a .eps file, 0 otherwise

		# update rows and columns
		self.row_col_update(self.nb_params)

		# new figure
		plt.figure()

	# update the number of rows and columns
	def row_col_update(self, nb_plot):

		if nb_plot == 1:
			self.nb_row = 1
			self.nb_col = 1

		elif nb_plot == 2:
			self.nb_row = 1
			self.nb_col = 2

		elif nb_plot == 3:
			self.nb_row = 1
			self.nb_col = 3

		elif nb_plot == 4:
			self.nb_row = 2
			self.nb_col = 2

		elif nb_plot == 5:
			self.nb_row = 2
			self.nb_col = 3

		elif nb_plot == 6:
			self.nb_row = 2
			self.nb_col = 3

		elif nb_plot == 7:
			self.nb_row = 2
			self.nb_col = 4

		elif nb_plot == 8:
			self.nb_row = 2
			self.nb_col = 4

		elif nb_plot == 9:
			self.nb_row = 3
			self.nb_col = 3

		elif nb_plot == 10:
			self.nb_row = 3
			self.nb_col = 4

		elif nb_plot == 11:
			self.nb_row = 3
			self.nb_col = 4

		elif nb_plot == 12:
			self.nb_row = 3
			self.nb_col = 4

		else:
			print('Error: rows, columns not defined for {} curves !'.format(nb_plot))
			exit(1)

	# add data with corresponding speed
	def add_data(self, new_data, new_real_speed, new_speed_ref):

		# safety
		if len(new_data.shape) != 2:
			print('Error: new_data must be a 2D tabular !')
			exit(1)

		if self.nb_params != new_data.shape[0]:
			print('Error: nb_params ({}) != new_data.shape[0] ({})'.format(self.nb_params, new_data.shape[0]))
			exit(1)

		if len(new_real_speed.shape) != 1:
			print('Error: new_real_speed must be a vector !')
			exit(1)

		if new_data.shape[1] != len(new_real_speed):
			print('Error: new_data.shape[1] ({}) != new_real_speed.shape[1] ({})'.format(new_data.shape[1], new_real_speed.shape[1]))
			exit(1)

		if not isFloat(new_speed_ref):
			print('Error: new_speed_ref must be a scalar !')
			exit(1)

		# increment count
		cur_nb_trials = new_data.shape[1]

		self.nb_sets += 1
		self.nb_tot += cur_nb_trials

		# add new data and speed
		self.data.append(new_data)
		self.speed_ref.append(new_speed_ref)
		self.nb_trials.append(cur_nb_trials)

		# data without scaling
		new_out = np.zeros((new_data.shape[0], new_data.shape[1]))

		for i in range(self.nb_params):
			for j in range(cur_nb_trials):
				new_out[i, j] = self.scaling(new_data[i, j], i)

		self.out.append(new_out)

		# averages and standard deviations
		new_out_av  = []
		new_out_std = []
		for i in range(new_out.shape[0]):
			new_out_av.append(np.mean(new_out[i]))
			new_out_std.append(np.std(new_out[i]))

		self.out_av.append(new_out_av)
		self.out_std.append(new_out_std)

		self.speed_av.append(np.mean(new_real_speed))
		self.speed_std.append(np.std(new_real_speed))

		self.speed_trial_av.append(new_real_speed)

	# check if a parameter ID is possible
	def check_param_id(self, param_id):
		if param_id < 0 or param_id >= self.nb_params:
			print('Error: param_id ({}) must be in [0;{}]'.format(param_id, self.nb_sets))
			exit(1)

	# perform scaling from normalized value
	def scaling(self, norm_val, param_id):

		# safety
		self.check_param_id(param_id)

		# scaling params
		min_scaling  = self.bounds[param_id][0]
		max_scaling  = self.bounds[param_id][1]
		diff_scaling = max_scaling -min_scaling

		# safety
		if diff_scaling <= 0.0:
			print('Error: diff_scaling ({}) must be strictly positive !'.format(diff_scaling))
			exit(1)

		# scaling computation
		return min_scaling + norm_val * diff_scaling

	# find the X-Y vectors for a given ID
	def X_Y_id(self, param_id):

		# safety
		self.check_param_id(param_id)

		# vectors initialization
		X = []
		Y = []

		# loop on all the speed references
		for i in range(self.nb_sets):

			# loop on the different trials for each speed
			for j in range(self.nb_trials[i]):

				X.append(self.speed_trial_av[i][j])
				Y.append(self.out[i][param_id][j])

		return X, Y

	# get the parameter ID, given its name
	def param_id_name(self, param_name):

		# loop on all the parameters
		for i in range(self.nb_params):
			if param_name == self.names[i]:
				return i

		# not found
		print('Error: parameter "{}" not found !'.format(param_name))
		exit(1)

	# get polynomial approximation
	def polynom_approx(self, order, X, Y, x):

		if order == 0:
			[alpha] = np.polyfit(X, Y, 0)

			y = alpha * np.ones((x.shape[0]))

			poly_string = 'y = {}'.format('{:.4f}'.format(alpha))

		elif order == 1:
			[alpha, beta_1] = np.polyfit(X, Y, 1)

			beta_2 = beta_1 + alpha * self.x_star

			y = alpha * (x - self.x_star) + beta_2

			poly_string = 'y = {} * (x - {}) + {}'.format('{:.4f}'.format(alpha), self.x_star, '{:.4f}'.format(beta_2))

		elif order == 2:
			[alpha, beta_1, gamma_1] = np.polyfit(X, Y, 2)

			beta_2  = beta_1  + 2.0 * alpha * self.x_star
			gamma_2 = gamma_1 + beta_2 * self.x_star - alpha * self.x_star * self.x_star

			diff_x = x - self.x_star

			y = alpha * diff_x * diff_x + beta_2 * diff_x + gamma_2

			poly_string = 'y = {} * (x - {})^2 + {} * (x - {}) + {}'.format('{:.4f}'.format(alpha), self.x_star, '{:.4f}'.format(beta_2), self.x_star, '{:.4f}'.format(gamma_2))

		else:
			print('Error: only polynomial forms from degree 0, 1 or 2 accepted')
			exit(1)

		return y, poly_string

	# computing the p-values with the 'lack-of-fit sum of squares' method
	def lack_of_fit(self, param_name, approx_order):

		param_id = self.param_id_name(param_name)

		# get X-Y points
		[X, Y] = self.X_Y_id(param_id)

		# tested speed
		x_approx = np.zeros((len(self.speed_av)))

		for i in range(len(self.speed_av)):
			x_approx[i] = self.speed_av[i]

		# get the approxmiation
		[y_approx, approx_string] = self.polynom_approx(approx_order, X, Y, x_approx)

		sse  = 0 # sum of squares error
		sslf = 0 # sum of squares lack of fit
		sspe = 0 # sum of squares pure error

		# loop on all the speed sets
		for i in range(self.nb_sets):

			sslf += self.nb_trials[i] * pow(self.out_av[i][param_id] - y_approx[i], 2)

			# loop on all the trials of each set
			for j in range(self.nb_trials[i]):

				sse += pow(self.out[i][param_id][j] - y_approx[i], 2)

				sspe += pow(self.out[i][param_id][j] - self.out_av[i][param_id], 2)

		# safety check
		if mt.fabs(sslf + sspe - sse) > 1.0e-10:
			print('lack-of-fit error: SSE != SSLF + SSPE')

		dof_lf = self.nb_sets - (approx_order+1) # dof associated with sslf
		dof_pe = self.nb_tot - self.nb_sets      # dof associated with sspe

		mslf = sslf / dof_lf # mean square lack of fit
		mspe = sspe / dof_pe # mean square pure error

		F = mslf / mspe # F* statistic

		# p-value: compare to a F ditribution with 'dof_lf' as numerator dof and 'dof_pe' as denominator dof
		p_val = scipy.stats.f.sf(F, dof_lf, dof_pe)

		return p_val

	# compute best polynomial order (orders 0, 1 or 2 possible)
	def lack_of_fit_012(self, param_name):

		res = []
		order_vec = [0, 1, 2]

		for i in range(len(order_vec)):
			res.append(self.lack_of_fit(param_name, order_vec[i]))

		thres_id = -1

		for i in range(len(order_vec)):
			if res[i] > self.p_thres:
				thres_id = order_vec[i]
				break

		res_string = "{} -> ".format(param_name)

		for i in range(len(order_vec)):
			res_string += '{}: {:.4f} %     '.format(order_vec[i], 100.0 * res[i])

		if thres_id == -1:
			res_string += '-> no nice fit'
		else:
			res_string += '-> correct fit with order {}'.format(thres_id)

		print(res_string)

	# plot for one parameter
	def param_plot(self, param_name):

		param_id = self.param_id_name(param_name)
		plot_fac = self.plot_fac[param_id]

		# get X-Y points
		[X, Y] = self.X_Y_id(param_id)

		# Y for scaling
		Y_sc = np.zeros((len(Y)))

		# unit conversions
		for i in range(len(Y)):
			Y_sc[i] = Y[i] * plot_fac

		# get the approxmiation
		x_approx = np.linspace(self.speed_av[0], self.speed_av[-1], 50)

		[_, approx_string] = self.polynom_approx(self.approx_order[param_id], X, Y, x_approx)
		[y_approx, _] = self.polynom_approx(self.approx_order[param_id], X, Y_sc, x_approx)

		# subplot division
		self.nb_subplot += 1
		if not self.flag_save:
			plt.subplot(self.nb_row, self.nb_col, self.nb_subplot)

		# plot approx
		plt.plot(x_approx, y_approx, 'k--')

		# plot standard deviations
		last_mean_speed = 0.0
		last_mean_out   = 0.0

		# loop on all the speed references
		for i in range(self.nb_sets):

			mean_speed = self.speed_av[i]
			mean_out   = self.out_av[i][param_id] * plot_fac

			std_speed = self.speed_std[i]
			std_out = self.out_std[i][param_id] * mt.fabs(plot_fac)

			plt.plot(mean_speed, mean_out, 'bo')

			# errorbar
			plt.errorbar(mean_speed, mean_out, xerr=std_speed/2.0, yerr=std_out/2.0, color='b', ecolor='b')

			if i >= 1:
				plt.plot([last_mean_speed, mean_speed], [last_mean_out, mean_out], 'b')

			last_mean_speed = mean_speed
			last_mean_out   = mean_out

		plt.xlim(self.plot_xlim[0], self.plot_xlim[1])
		plt.ylim(self.plot_ylim[param_id][0], self.plot_ylim[param_id][1])

		if self.flag_save:
			ax = plt.gca()

			# labels
			plt.xlabel(r'$\boldsymbol{speed~(m/s)}$', size=20)
			plt.ylabel(self.labels[param_id], size=20)

			# label space
			ax.xaxis.labelpad = 10
			ax.yaxis.labelpad = 10

			# hide right and top spines
			ax.spines['right'].set_visible(False)
			ax.spines['top'].set_visible(False)

			# only show ticks on left and bottom spines
			ax.xaxis.set_ticks_position('bottom')
			ax.yaxis.set_ticks_position('left')

			# ticks parameters
			plt.tick_params(axis='both', which='major', labelsize=20, direction='out', width=2)

			# save figure
			plt.savefig('{}/sp_adapt_{}.eps'.format(self.output_folder, param_name), bbox_inches='tight', pad_inches=0.25)

			# new figure
			plt.figure()

		else:
			# plot title
			plt.title(self.labels[param_id], size=10)

			# print approximation
			print('{} -> {}'.format(self.names[param_id], approx_string))
