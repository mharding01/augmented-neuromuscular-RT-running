# This file studies the walking features related to the running speed references

import numpy as np
import matplotlib.pyplot as plt

# speed references [m/s]
v_ref = np.array([1.25, 1.3, 1.35, 1.4, 1.45, 1.5, 1.55, 1.6])

# length of the 'v_ref' vector
v_len = len(v_ref)

# number of trials
nb_trials = 10

# lines legend:
#
#  0 : speed (m/s)
#  1 : stride period (s)
#  2 : stride length (m)
#  3 : flight cycle (%)
#  4 : metabolic energy for the leg muscles (J/(kg*m))

data_125 = np.array([
	[ 1.31623  , 1.30535  , 1.3043   , 1.30765  , 1.31432  , 1.32101  , 1.3101   , 1.3124   , 1.31234  , 1.31535  ],
	[ 0.517245 , 0.518734 , 0.519724 , 0.518131 , 0.517942 , 0.517202 , 0.517901 , 0.518269 , 0.517801 , 0.517194 ],
	[ 0.681729 , 0.677118 , 0.676206 , 0.678952 , 0.680731 , 0.681697 , 0.679768 , 0.678796 , 0.679895 , 0.681944 ],
	[ 29.0884  , 28.7843  , 28.5289  , 28.4913  , 29.132   , 28.3094  , 28.8314  , 30.4833  , 28.4534  , 32.0669  ],
	[ 8.9572   , 9.04906  , 9.08806  , 9.02207  , 8.98516  , 8.95775  , 8.99087  , 9.02583  , 8.97952  , 8.96706  ],
])

data_130 = np.array([
	[ 1.33902  , 1.33863  , 1.33701  , 1.32916  , 1.33387  , 1.34018  , 1.33818  , 1.33352  , 1.34056  , 1.33996  ],
	[ 0.513143 , 0.512684 , 0.513184 , 0.514314 , 0.513789 , 0.513568 , 0.512974 , 0.513723 , 0.51284  , 0.513565 ],
	[ 0.686702 , 0.687499 , 0.687441 , 0.684288 , 0.685454 , 0.686194 , 0.686582 , 0.685758 , 0.687485 , 0.686038 ],
	[ 29.1135  , 30.4569  , 29.0317  , 28.6604  , 30.9173  , 29.0753  , 28.7991  , 30.1864  , 30.1695  , 29.1301  ],
	[ 8.8572   , 8.83909  , 8.85775  , 8.89157  , 8.87577  , 8.86474  , 8.85781  , 8.86833  , 8.8469   , 8.86622  ]
])

data_135 = np.array([
	[ 1.36187  , 1.36309  , 1.36276  , 1.3647   , 1.36364  , 1.36759  , 1.35765  , 1.36368  , 1.36448  , 1.36463  ],
	[ 0.508142 , 0.508353 , 0.508534 , 0.508133 , 0.508753 , 0.507793 , 0.509191 , 0.508349 , 0.5085   , 0.508175 ],
	[ 0.693464 , 0.693291 , 0.693067 , 0.693868 , 0.692285 , 0.694756 , 0.691211 , 0.693132 , 0.692726 , 0.693652 ],
	[ 30.3866  , 30.5729  , 29.1394  , 30.6386  , 29.7625  , 28.9371  , 29.7298  , 28.7346  , 29.4094  , 29.5627  ],
	[ 8.71002  , 8.71585  , 8.71619  , 8.7154   , 8.74037  , 8.68154  , 8.75519  , 8.71718  , 8.7367   , 8.70973  ]
])

data_140 = np.array([
	[ 1.3978   , 1.39534  , 1.39532  , 1.39536  , 1.39501  , 1.39141  , 1.39552  , 1.39984  , 1.38987  , 1.39861  ],
	[ 0.502475 , 0.502111 , 0.502595 , 0.502394 , 0.502327 , 0.502561 , 0.502606 , 0.502177 , 0.502611 , 0.501941 ],
	[ 0.701602 , 0.701614 , 0.700901 , 0.701483 , 0.701023 , 0.700907 , 0.700003 , 0.702209 , 0.700522 , 0.702334 ],
	[ 30.0954  , 30.8397  , 30.7648  , 31.6963  , 29.8304  , 29.5175  , 30.3167  , 30.2209  , 29.2762  , 29.4931  ],
	[ 8.56476  , 8.55876  , 8.57634  , 8.56996  , 8.56598  , 8.57345  , 8.58742  , 8.55425  , 8.58458  , 8.5466   ]
])

data_145 = np.array([
	[ 1.43839  , 1.4349   , 1.43432  , 1.42904  , 1.4355   , 1.43487  , 1.437    , 1.43772  , 1.4298   , 1.42911  ],
	[ 0.495342 , 0.495282 , 0.495666 , 0.495395 , 0.495417 , 0.495205 , 0.495153 , 0.495005 , 0.495717 , 0.495581 ],
	[ 0.710561 , 0.711241 , 0.710053 , 0.709818 , 0.710597 , 0.711761 , 0.711201 , 0.711286 , 0.710379 , 0.710306 ],
	[ 30.5256  , 28.7264  , 29.8028  , 29.4671  , 30.3519  , 28.388   , 29.3392  , 32.1882  , 30.5928  , 29.6501  ],
	[ 8.43751  , 8.41778  , 8.44493  , 8.44022  , 8.43575  , 8.40803  , 8.42747  , 8.42555  , 8.43901  , 8.43477  ],
])

data_150 = np.array([
	[ 1.46002  , 1.46102  , 1.46488  , 1.48356  , 1.47095  , 1.46059  , 1.46187  , 1.46151  , 1.46152  , 1.46859  ],
	[ 0.490619 , 0.492983 , 0.491102 , 0.490809 , 0.488889 , 0.492236 , 0.49283  , 0.491914 , 0.493069 , 0.49198  ],
	[ 0.717743 , 0.716924 , 0.718405 , 0.723134 , 0.719812 , 0.714004 , 0.717207 , 0.719942 , 0.719064 , 0.719272 ],
	[ 29.4475  , 31.8782  , 31.3921  , 31.2034  , 30.7228  , 31.3901  , 31.0383  , 31.3603  , 32.1213  , 30.8453  ],
	[ 8.31714  , 8.40076  , 8.34546  , 8.26856  , 8.32422  , 8.37551  , 8.38617  , 8.31799  , 8.34656  , 8.37639  ]
])

data_155 = np.array([
	[ 1.48054  , 1.48157 , 1.48529  , 1.5044   , 1.48116  , 1.48676  , 1.47994  , 1.46924  , 1.47405  , 1.48806  ],
	[ 0.482723 , 0.48216 , 0.481601 , 0.482928 , 0.483395 , 0.480922 , 0.480737 , 0.48189  , 0.482758 , 0.482498 ],
	[ 0.716448 , 0.72456 , 0.7171   , 0.719123 , 0.717056 , 0.718341 , 0.714166 , 0.709982 , 0.723013 , 0.718684 ],
	[ 31.2244  , 31.6676 , 31.9193  , 29.6743  , 32.3092  , 32.1401  , 32.1673  , 31.6105  , 31.378   , 29.8287  ],
	[ 8.3285   , 8.23931 , 8.29495  , 8.287    , 8.30518  , 8.27109  , 8.32075  , 8.38855  , 8.24122  , 8.31739  ]
])

data_160 = np.array([
	[ 1.60443  , 1.61319  , 1.5586   , 1.58398  , 1.65768  , 1.54488  , 1.58262  , 1.55024  , 1.59038  , 1.55912  ],
	[ 0.470488 , 0.470366 , 0.471765 , 0.472085 , 0.470612 , 0.471399 , 0.473032 , 0.473696 , 0.470524 , 0.472431 ],
	[ 0.743975 , 0.749858 , 0.744046 , 0.746152 , 0.750438 , 0.738994 , 0.743512 , 0.735599 , 0.74985  , 0.745262 ],
	[ 29.9602  , 30.2513  , 30.1548  , 29.9455  , 29.1979  , 29.9871  , 30.7045  , 32.7168  , 29.4298  , 30.756   ],
	[ 7.91565  , 7.87969  , 7.95251  , 7.93566  , 7.87376  , 7.96708  , 7.98537  , 8.05415  , 7.86499  , 7.94502  ]
])

data = np.array([data_125, data_130, data_135, data_140, data_145, data_150, data_155, data_160])

# compute stride frequencies
data_freq = np.zeros((v_len, nb_trials))

for i in range(v_len):
	for j in range(nb_trials):
		data_freq[i][j] = 1.0 / data[i][1][j]

# output folder
output_folder = './'

# 1 to save the figure, 0 otherwise
save_fig = 1

speed_mean = np.zeros((v_len))
speed_std  = np.zeros((v_len))

period_mean = np.zeros((v_len))
period_std  = np.zeros((v_len))

freq_mean = np.zeros((v_len))
freq_std  = np.zeros((v_len))

length_mean = np.zeros((v_len))
length_std  = np.zeros((v_len))

flight_mean = np.zeros((v_len))
flight_std  = np.zeros((v_len))

energy_mean = np.zeros((v_len))
energy_std  = np.zeros((v_len))

#  0 : speed (m/s)
#  1 : stride period (s)
#  2 : stride length (m)
#  3 : flight cycle (%)
#  4 : metabolic energy for the leg muscles (J/(kg*m))

for i in range(v_len):
	speed_mean[i] = np.mean(data[i][0])
	speed_std[i]  = np.std(data[i][0])

	period_mean[i] = np.mean(data[i][1])
	period_std[i]  = np.std(data[i][1])

	length_mean[i] = np.mean(data[i][2])
	length_std[i]  = np.std(data[i][2])

	flight_mean[i] = np.mean(data[i][3])
	flight_std[i]  = np.std(data[i][3])

	energy_mean[i] = np.mean(data[i][4])
	energy_std[i]  = np.std(data[i][4])

	freq_mean[i] = np.mean(data_freq[i])
	freq_std[i]  = np.std(data_freq[i])


# get a figure
def get_figure(var_mean, var_std, y_label, prefix_name):

	plt.figure(figsize=(10,5))
	ax = plt.gca()

	plt.errorbar(speed_mean, var_mean, xerr=speed_std, yerr=var_std, color='b', ecolor='b', fmt="o")
	plt.plot(speed_mean, var_mean, 'b--', lw=1)

	plt.xlabel('speed (m/s)', size=20)
	plt.ylabel(y_label, size=20)
	plt.xlim([1.275, 1.625])
	#plt.ylim([0.47, 0.52])

	# hide right and top spines
	ax.spines['right'].set_visible(False)
	ax.spines['top'].set_visible(False)

	# only show ticks on left and bottom spines
	ax.xaxis.set_ticks_position('bottom')
	ax.yaxis.set_ticks_position('left')

	# ticks parameters
	plt.tick_params(axis='both', which='major', labelsize=15, direction='out', width=2)

	if save_fig:
		plt.savefig('{}/{}_run_feature.eps'.format(output_folder, prefix_name), bbox_inches='tight', pad_inches=0.25)

# get all figures
get_figure(period_mean, period_std, 'stride period (s)', 'period')
get_figure(freq_mean  , freq_std  , 'stride frequency (Hz)', 'freq')
get_figure(length_mean, length_std, 'stride length (m)', 'length')
get_figure(flight_mean, flight_std, 'flying phase (%)', 'flight')
get_figure(energy_mean, energy_std, 'metabolic energy (J /m kg)', 'energy')

plt.show()
