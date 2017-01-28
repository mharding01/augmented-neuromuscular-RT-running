import numpy as np

from scipy import stats
import matplotlib.pyplot as plt
import os

metrics = ["energy", "speed", "period", "length", "one leg stance phase", 
            "double support", "tot flight"];

def find(name, path):
    for root, dirs, files in os.walk(path):
        if name in files:
            return os.path.join(root, name);
    return None;

def parse_substring_between(ch1, ch2, string):
    start = string.find(ch1); end = string.find(ch2);
    return string[ (start + len(ch1)):end ];

def parse_results_file(filename, speed, trial, data):
    filetype = "OptiResults" if (filename.find('OptiResults') != -1) else "out_norm";
    with open(filename, 'r') as f:

        # loop over lines until first "g_osc->set*"
        p_count = 0;
        
        for line in f:
            if line.find("g_osc->") != -1:
                # For parameter # <p_count> extract value
                raw_val_s = parse_substring_between('(', ')', line);
                val = float( raw_val_s.strip() );
                data[speed][filetype][p_count][trial] = val;
                p_count += 1;
            for metric in metrics:
                if line.find(metric) != -1:
                    # Parse metric (val, units)
                    raw_s = parse_substring_between(':', line[-1], line).strip();
                    val_s, units = raw_s.split(' ');
                    data[speed][filetype][metric][trial] = (float(val_s), units);
        assert(p_count == 11);

def extract_data_from_dir(directory, c_path, data):
    # assert directory exists
    assert(c_path == os.getcwd());
    assert(os.path.exists(os.path.join(c_path, directory)));
   
    # assert speed is one we're expecting
    speed = float(directory.split('kk')[0]);
    assert(speed in data);

    # assumes nb_trials < 10
    trial = int(directory[-1]);

    # find results files
    dir_path = os.path.join(c_path, directory);
    OR_f = find("OptiResults.cc", dir_path);
    ON_f = find("out_norm.txt", dir_path);
    assert(not (OR_f == None));
    assert(not (ON_f == None));
   
    # Parse these files
    parse_results_file(OR_f, speed, trial-1, data);
    parse_results_file(ON_f, speed, trial-1, data);
    return;

def recenter_poly(coeffs, x_0):
    # Coeffs is polynomial coefficients centered around x=0
    # x_0 is new center of polynomial - i.e. (x-x_0) is x* 
    if (len(coeffs) == 2):
        # Linear adjustment: y = mx + b
        m, b = coeffs;
        new_p = [m, b + m*x_0];
        p1 = np.poly1d(coeffs); p2 = np.poly1d(new_p);
        return new_p

    if (len(coeffs) == 3):
        # Quadratic recentering: y = ax**2 + bx + c
        a, b, c = coeffs;
        d = a;
        e = b + 2*a*x_0;
        f = a*x_0**2 + b*x_0 + c;
        new_p = [d, e, f];
        p1 = np.poly1d(coeffs); p2 = np.poly1d(new_p);
        return new_p;

# var is either a 'param' (an index) or a metric string
def make_var_plot(var, data_d, param_meta, regr_d, recenter_x = 0.0, out_norm_flag = False):
    filetype = "OptiResults" if not out_norm_flag else "out_norm"
    metric_flag = (var in metrics);
    var_name, units = (var, "") if (metric_flag) else param_meta[var];
    # x-axis: speed, y-axis: var value
    x = [];
    y = [];
    x_mean = []; y_mean = []; x_std = []; y_std = [];
    for speed in data_d:
        # Lists per speed for averaging
        x_speed = [];
        y_speed = [];   
        # For convenience
        sp_results = data_d[speed][filetype];
        trials = sp_results[var];
        for trial_i in range(len(trials)):
            trial_val = trials[trial_i];
            if (trial_val != None): # Ignore missing data
                print sp_results["speed"], trial_i
                actual_speed, _units = sp_results["speed"][trial_i];
                if (metric_flag):
                    # Re-read trials entries as metric tuple: (val, units)
                    trial_val, _units = trials[trial_i];
                    if (units == ""): units = _units;

                # Build plotting data lists
                x.append(actual_speed);
                y.append(trial_val);
                # Append to averaging lists
                x_speed.append(actual_speed);
                y_speed.append(trial_val);  
    
        # Compute mean and standard dev's over trials for this speed
        x_mean.append(np.mean(x_speed));
        y_mean.append(np.mean(y_speed));
        x_std.append(np.std(x_speed));
        y_std.append(np.std(y_speed));

    # modify y_std 
    y_std = 1.0 * np.asarray(y_std);

    # Find polyfit
    m, b, r, p, std_err = stats.linregress(x, y);
    p2, residuals, _, _, _ = np.polyfit(x,y, 2, full=True);
    poly_l = np.poly1d([m,b]);
    poly_q = np.poly1d(p2);

    # Find polynomial centered around recenter_x
    re_l_coeffs = recenter_poly([m,b], recenter_x);
    re_q_coeffs = recenter_poly(p2, recenter_x);

    # Save regression data
    regr_d[var_name] = [ [m, b], list(p2), 
                            (recenter_x, re_l_coeffs), 
                            (recenter_x, re_q_coeffs)];

    # Create new figure
    plt.figure();
    xp = np.linspace(1.2, 1.8, 100);
    _ = plt.plot(xp, poly_l(xp), '-');
    _ = plt.plot(xp, poly_q(xp), '-');
    plt.errorbar(x_mean, y_mean, yerr=y_std, xerr=x_std,
                    linestyle="None", marker='^');
    plt.scatter(x, y);
    #fit_string = "Linear fit: R-squared [%.4f] / Quadratic fit: Residuals [%.4f]" % (r**2, residuals);
    fit_string = "";
    title_string = "" + var_name + "\n" + fit_string;
    plt.xlabel('Speed [m/s]');
    plt.ylabel(var_name + " " + units);
    fig_name = ""+var_name+"-fig";
    plt.savefig(fig_name + ".png", bbox_inches='tight');
    print "%s.png saved!" % fig_name;
    #plt.show();
    return;

def is_float_str(s):
    try:
        float(s);
        return True;
    except ValueError:
        return False;

if __name__ == "__main__":
    cur_path = os.getcwd();
    dirs = [d for d in os.listdir(cur_path) if os.path.isdir(os.path.join(cur_path, d))];
    print "Starting ...\nDirectories are:", dirs
    nb_params = 11;
    nb_trials = 5;
    
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
    # Reads directory names: <speed>"ank"<trial number>
    speeds = [];
    speed_dirs = [];
    for d in dirs:
        sp_str = d.split('kk')[0];
        if (is_float_str(sp_str)): 
            speeds.append(float(sp_str));
            speed_dirs.append(d);

    # construct empty data dictionary
    # data stored in dictionary of format: 
    # d[speed][file - "OptiResults" or "out_norm"][var# (or metrics)] = [5 trials] or metric value;
    data_d = dict();
    for speed in speeds:
        data_d[speed] = dict();
        data_d[speed]["OptiResults"] = dict();
        data_d[speed]["out_norm"] = dict();
        for param in range(nb_params):
            data_d[speed]["OptiResults"][param] = [None]*nb_trials;
            data_d[speed]["out_norm"][param] = [None]*nb_trials;
        for metric in metrics:
            data_d[speed]["OptiResults"][metric] = [None]*nb_trials;
            data_d[speed]["out_norm"][metric] = [None]*nb_trials;


    print "Dictionary created";
    # read data into data_d
    for sp_d in speed_dirs:
        extract_data_from_dir(sp_d, cur_path, data_d);
        print "Data extracted from directory:", sp_d
   
    # plot scatter plots and regression data for each variable
    regr_d = dict(); # regr_d[param_name or metric] = [ (lin. coeffients), (quad. coeffs),
                     #                (recentered lin. coeffs), (recentered quad. coeffs)]
    v_ref = 1.6;
    for param in range(nb_params):
        make_var_plot(param, data_d, param_meta, regr_d, recenter_x = v_ref, out_norm_flag = False);
    for metric in metrics:
        make_var_plot(metric, data_d, param_meta, regr_d, recenter_x = v_ref, out_norm_flag = False);

    import json
    with open("dict.txt", 'w') as f:
        print "Dumping data dictionary (JSON) to dict.txt ..."
        json.dump(data_d, f, indent=1);

    with open("regr.txt", 'w') as f:
        print "Dumping regression data (JSON) to regr.txt ..."
        json.dump(regr_d, f, indent=1);
    print "Done!";
