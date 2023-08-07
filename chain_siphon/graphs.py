import os, sys
from matplotlib import pyplot as plt
import numpy as np
from scipy.stats import norm
from siphon_utils import *

SHOW = False

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CHAIN_DIR = os.path.join(DIR, 'chain_siphon')
ALL = []
RT = 'output'
ROOT = os.path.join(CHAIN_DIR, RT)

CONF_INT = -1  # either the percent confidence interval, or negative number for # of stdev to show
EXCLUDED = [
    # 'max',
    # 'control',
    # 'chain',
    # 'LJP',
    'leader',
]

name_mapping = {'chain': 'Chain Siphon',
                'control': 'Control',
                'LJP': 'LJP',
                'leader': 'Leader',
                'max': 'Theoretical Max'}

OUTPUT_DIR = os.path.join(ROOT, 'plots')

VALUES = range(1, 31)
ROUND = 1
CARE = -1  # use numbers in linear reg if >CARE

for (plotting, PROP) in (('failed', False), ('failed', True), ('successful', False), ('successful', True)):
    print("plotting " + plotting + ' blimps, proportion is', PROP)
    legend_entries = 0
    labeled_bars = False

    if plotting == 'failed':
        plotting = 'failed'
        y_ax = 'stuck'
    else:
        plotting = 'successful'
        y_ax = 'successful'

    for folder in os.listdir(ROOT):
        if folder in EXCLUDED:
            continue
        folder_dir = os.path.join(ROOT, folder)

        data_dict, data = datadict_from_folder(folder_dir)
        if data_dict is None:
            continue

        keys = list(data_dict.keys())
        keys.sort()

        values = np.array([data_dict[entry][plotting] for entry in keys])
        if PROP:
            y = values/keys
        else:
            y = values
        var = np.array([data_dict[entry]['var'] for entry in keys])
        trials = np.array([data_dict[entry]['trials'] for entry in keys])
        stdev = np.sqrt(var)
        # VALUES=DATA['num_agents']

        label = folder
        if label in name_mapping:
            label = name_mapping[label]
        print("\tplotting:", label)

        plt.plot(keys, y, alpha=1, label=label)
        legend_entries += 1
        # leg.append(folder)

        bar_label = None
        if CONF_INT > 0:
            percentile = 1 - (1 - CONF_INT/100)/2
            # since double sided

            # 95% confidence interval, subtract 1 from num trials since 1 DOF lost for variance
            conf = (stdev/np.sqrt(trials - 1))*norm.ppf(percentile)
            if PROP:
                conf = conf/keys
            if not labeled_bars:
                labeled_bars = True
                bar_label = str(CONF_INT) + '% conf. int'
            low = np.max((y - conf, [0 for _ in range(len(y))]), axis=0)
            high = np.min((y + conf, [(1 if PROP else i) for i in range(1, 1 + len(y))]), axis=0)
            plt.fill_between(keys, low, high,
                             alpha=.35,
                             color='gray', label=bar_label
                             )
            if bar_label is not None:
                legend_entries += 1
            ALL.append(y)
        elif CONF_INT < 0:
            conf = stdev*np.abs(CONF_INT)
            if PROP:
                conf = conf/keys

            if not labeled_bars:
                labeled_bars = True
                bar_label = '$\pm' + str(abs(CONF_INT)) + '$ stdev'

            low = np.max((y - conf, [0 for _ in range(len(y))]), axis=0)
            high = np.min((y + conf, [(1 if PROP else i) for i in range(1, 1 + len(y))]), axis=0)
            plt.fill_between(keys, low, high,
                             alpha=.35,
                             #color='gray', label=bar_label
                             )
            if bar_label is not None:
                legend_entries += 1
        if False:
            XY = []
            for x, s in zip(keys, values):
                if s > CARE:
                    XY.append([x, s])
                else:
                    XY = []
            XY = np.array(XY)
            Y = XY[:, [1]]
            X = np.concatenate((np.ones((len(XY), 1)), XY[:, [0]]), axis=1)
            result = np.linalg.lstsq(X, Y, rcond=None)
            b = result[0][0][0]
            m = result[0][1][0]

            guess = np.array([[i*m + b] for i in VALUES])

            plt.plot(VALUES, guess.reshape(-1)/(np.array(VALUES) if PROP else 1), '--' if folder == "CHAINS" else ':',
                     alpha=.5, color='purple',
                     label=('y = {0}*x' + (' + ' if b > 0 else ' ') + '{1}').format(round(m, ROUND), round(b, ROUND)))
            legend_entries += 1
    if 'max' not in EXCLUDED:
        plt.plot(VALUES, [(1 if PROP else i) for i in VALUES], '--',
                 alpha=.5,
                 label=name_mapping['max'] if 'max' in name_mapping else 'max'
                 )
        legend_entries += 1
    plt.xlabel('Number of agents')

    plt.ylabel(('Proportion of ' if PROP else '') + y_ax + ' blimps')
    plt.ylim((0, 1.1 if PROP else plt.ylim()[1]))
    plt.legend(  # + leg_fill[0:1],
        # loc=('lower right' if PROP else 'upper left')
        # loc='center right', bbox_to_anchor=(1.2, .5),ncol=3,
        loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=int(np.ceil(legend_entries/2))
    )
    save_file = os.path.join(OUTPUT_DIR, ('prop_' if PROP else '') + y_ax + '_rate.png')
    if not os.path.exists(os.path.dirname(save_file)):
        os.makedirs(os.path.dirname(save_file))
    plt.savefig(save_file)
    if SHOW:
        plt.show()
    plt.close()

for folder in os.listdir(ROOT):
    if folder in EXCLUDED:
        continue
    folder_dir = os.path.join(ROOT, folder)

    data_dict, data = datadict_from_folder(folder_dir)
    if data_dict is None:
        continue
    
    keys = list(data_dict.keys())
    keys.sort()

    trials = np.array([data_dict[entry]['trials'] for entry in keys])
    
    label = folder
    if label in name_mapping:
        label = name_mapping[label]

    plt.plot(keys, trials, alpha=.5, label=label)

plt.xlabel('Number of agents')
plt.xlim(0,plt.xlim()[1])
plt.ylim(0,plt.ylim()[1])
plt.ylabel('Trials run')

plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=4)
save_file = os.path.join(OUTPUT_DIR, 'trials.png')
if not os.path.exists(os.path.dirname(save_file)):
    os.makedirs(os.path.dirname(save_file))
plt.savefig(save_file)
plt.close()
