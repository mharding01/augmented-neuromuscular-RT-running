import numpy as np
import json 
import AllData as ad 

def get_speed_data(data_d, speed, ordered_params):
    # convert speed to string (key)
    speed = str(speed);
    assert(speed in data_d);

    speed_data = data_d[speed]["out_norm"]; 
    num_params = len(ordered_params);
    num_trials = len(speed_data["0"]);
    vals = [];
    speeds = [];

    for p in ordered_params:
        vals.append( speed_data[p] );
    for sp_trial in speed_data["speed"]:
        if (sp_trial != None):
            speeds.append(sp_trial[0]);
        else: 
            speeds.append(None);

    return np.asarray(vals), np.asarray(speeds);

if __name__ == "__main__":
    # data stored in dictionary of format: 
    # d["<speed>"][file - "OptiResults" or "out_norm"]["<var#>" (or metrics)] = [5 trials] or metric value;
    data = dict();
    with open("dict.txt", "r") as f:
        data = json.load(f);
        print ("Read in most recent 'dict.txt' successfully!");
    
    # Parameter meta data
    param_meta = [(r'$\tau$', "[s]"),
                  (r'$\theta_{trunk}$', "[rad]"),
                  (r'$\theta_{hip}$', "[rad]"),
                  (r'$k_{HFL_1}$', "[.]"),
                  (r'$k_{HFL_2}$', "[.]"),
                  (r'$k_{HAM}$', "[.]"),
                  (r'$G_{sol}$', "[.]"),
                  (r'$G_{sol_{ta}}$', "[.]"),
                  (r'$G_{gas}$', "[.]"),
                  (r'$G_{vas}$', "[.]"),
                  (r'$k_\theta$', "[rad]")];
    metrics = ["energy", "speed", "period", "length", "one leg stance phase", 
            "double support", "tot flight"];

    # Ordered parameters (keys of data) - regular params's keys are 0-indexed
    ord_params = [str(i) for i in range(len(param_meta))];

    # names of the variables 
    names = ['tau', 'theta_trunk', 'theta_hip', 'k_HFL1', 'k_HFL2',
                'k_HAM', 'G_sol', 'G_sol_ta', 'G_gas', 'G_vas', 'k_theta'];

    # plot factors for unit conversions
    plot_fac = [1.0] * len(names);

    # labels for plot 
    labels = param_meta;

    # approximation order
    approx_order = [1]*len(names);

    # parameter bounds
    bounds = np.array([
        [ 0.010 , 0.15   ] ,# tau
        [ 0.0  , 0.25   ] ,#theta_trunk
        [ 0.005 , 0.9   ] ,#theta_hip
        [ 0.3  , 8.1    ] ,#k_HFLrun1
        [ 1.0  , 12.0   ] ,#k_HFLrun2
        [ 0.3  , 7.0   ] ,#k_HAMrun
        [ 0.60  , 5.0  ] ,#G_sol
        [ 0.4  , 7.0   ] ,#G_sol_ta
        [ 0.0 , 20.0   ] ,#G_gas
        [ 0.82 , 5.0   ] ,#G_vas
        [ 1.0 , 15.0   ]#k_theta
    ])
    
    # xlim values
    plot_xlim  = np.array([ 1.2 , 1.8 ])

    # ylim values - TODO: not used
    plot_ylim = np.array([
        [ 1.4   , 2.2  ] ,
        [ 3.0   , 6.0  ] ,
        [ 1.0   , 3.0  ] ,
        [ 0.03  , 0.13 ] ,
        [ 1.0   , 4.5  ] ,
        [ 0.4   , 1.0  ] ,
        [ 0.0   , 0.12 ] ,
        [ 6.0   , 16.0 ] ,
        [ 2.1   , 3.1  ] ,
        [ 7.0   , 11.0 ] ,
        [ 73.0  , 92.0 ]
    ])

    # reference speed
    x_star = 1.6;

    # p-value threshold
    p_thres = 0.05

    # output folder
    output_folder = '~/BIOROB/coman_matthew_new/workR/Optis_2.0/more'

    all_data = ad.AllData(names, plot_fac, labels, approx_order, bounds,
                            x_star, p_thres, plot_xlim, plot_ylim, output_folder);

    data_130, speed_130 = get_speed_data(data, 1.3, ord_params);
    print data_130
    print speed_130
    data_135, speed_135 = get_speed_data(data, 1.35, ord_params);
    data_140, speed_140 = get_speed_data(data, 1.4, ord_params);
    data_145, speed_145 = get_speed_data(data, 1.45, ord_params);
    data_150, speed_150 = get_speed_data(data, 1.5, ord_params);
    data_155, speed_155 = get_speed_data(data, 1.55, ord_params);
    data_160, speed_160 = get_speed_data(data, 1.6, ord_params);
    data_165, speed_165 = get_speed_data(data, 1.65, ord_params);
    data_170, speed_170 = get_speed_data(data, 1.7, ord_params);

    # add data
    all_data.add_data(data_130, speed_130, 1.30)
    all_data.add_data(data_135, speed_135, 1.35)
    all_data.add_data(data_140, speed_140, 1.40)
    all_data.add_data(data_145, speed_145, 1.45 )
    all_data.add_data(data_150, speed_150, 1.50)
    all_data.add_data(data_155, speed_155, 1.55)
    all_data.add_data(data_160, speed_160, 1.60)
    all_data.add_data(data_165, speed_165, 1.65)
    all_data.add_data(data_170, speed_170, 1.70)


    # option to save the graphs
    flag_save = 0; 
    all_data.flag_save = flag_save

    # polynomial order
    if not flag_save:
        print('')
        all_data.lack_of_fit_012('tau')
        all_data.lack_of_fit_012('theta_trunk')
        all_data.lack_of_fit_012('theta_hip')
        all_data.lack_of_fit_012('k_HFL1')
        all_data.lack_of_fit_012('k_HFL2')
        all_data.lack_of_fit_012('k_HAM')
        all_data.lack_of_fit_012('G_sol')
        all_data.lack_of_fit_012('G_sol_ta')
        all_data.lack_of_fit_012('G_gas')
        all_data.lack_of_fit_012('G_vas')
        all_data.lack_of_fit_012('k_theta')

