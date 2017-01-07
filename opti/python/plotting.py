import numpy as np
from scipy import stats
import matplotlib.pyplot as plt
import os

def find(name, path):
    for root, dirs, files in os.walk(path):
        if name in files:
            return os.path.join(root, name);
    return None;

def parse_results_file(filename, speed, trial, data):
    filetype = "OptiResults" if (filename.find('OptiResults') != -1) else "out_norm";
    print "Reading ... (%s)" % filename, "is it ", filetype, "?"
    with open(filename, 'r') as f:

        # loop over lines until first "g_osc->set*"
        p_count = 0;
        for line in f:
            if line.find("g_osc->") != -1:
                # For parameter # <p_count> extract value
                start = line.find('('); end = line.find(')');
                val = float( line[ (start + 1):end].strip());
                data[p_count][speed][filetype][trial] = val;
                p_count += 1;
        assert(p_count == 9);

def extract_data_from_dir(directory, c_path, data):
    # assert directory exists
    assert(c_path == os.getcwd());
    assert(os.path.exists(os.path.join(c_path, directory)));
   
    # assert speed is one we're expecting
    speed = float(directory.split('a')[0]);
    assert(speed in data[0]);

    # assumes nb_trials < 10
    trial = int(directory[-1]);

    # find results files
    dir_path = os.path.join(c_path, directory);
    OR_f = find("OptiResults.cc", dir_path);
    ON_f = find("out_norm.txt", dir_path);
    assert(not (OR_f == None));
    assert(not (ON_f == None));
   
    # Parse these files
    print OR_f, ON_f;
    parse_results_file(OR_f, speed, trial-1, data);
    parse_results_file(ON_f, speed, trial-1, data);
    return;

def make_param_plot(param, data_d, out_norm_flag = False):
    filetype = "OptiResults" if not out_norm_flag else "out_norm"
    # x-axis: speed, y-axis: param value
    x = [];
    y = [];
    x_mean = []; y_mean = []; y_std = [];
    for speed in data_d[param]:
        y_speed = [];   # List only for this speed
        for trial_val in data_d[param][speed][filetype]:
            if (trial_val != None): # Ignore missing data
                x.append(speed);
                y_speed.append(trial_val);  
                y.append(trial_val);
        x_mean.append(speed);
        y_mean.append(np.mean(y_speed));
        y_std.append(np.std(y_speed));

    # modify y_std to be half y_std
    y_std = 1.0 * np.asarray(y_std);

    # Find polyfit
    m, b, r, p, std_err = stats.linregress(x, y);
    poly = np.poly1d([m, b]);
    plt.figure();
    xp = np.linspace(1.2, 2.1, 100);
    _ = plt.plot(xp, poly(xp), '-');
    plt.errorbar(x_mean, y_mean, y_std, linestyle="None", marker='^');
    plt.title("Param " + str(param) + " scatter plot and fit, R-squared " + str(r**2));
    plt.xlabel('Speed [m/s]');
    plt.ylabel('Parameter optimized value');
    plt.show();
    return;

if __name__ == "__main__":
    cur_path = os.getcwd();
    dirs = [d for d in os.listdir(cur_path) if os.path.isdir(os.path.join(cur_path, d))];
    print "Starting ...\nDirectories are:", dirs
    nb_params = 9;
    nb_trials = 10;

    # Reads directory names: <speed>"ank"<trial number>
    speeds = [float(d.split('a')[0]) for d in dirs];

    # construct empty data dictionary
    # data stored in dictionary of format: d[var#][speed][file - "OptiResults" or "outnorm"] = [5 trials];
    data_d = dict();
    for param in range(nb_params):
        data_d[param] = dict();
        for speed in speeds:
            data_d[param][speed] = {"OptiResults":[None]*nb_trials, 
                                    "out_norm":[None]*nb_trials};
    print "Dictionary created";

    # read data into data_d
    for d in dirs:
        extract_data_from_dir(d, cur_path, data_d);
        print "Data extracted from directory:", d
   
    # plot scatter plots and regression data for each variable
    for param in range(nb_params):
        make_param_plot(param, data_d, out_norm_flag = True);
