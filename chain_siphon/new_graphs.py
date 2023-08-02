import csv, os, sys
from collections import defaultdict

from matplotlib import pyplot as plt
import numpy as np
from scipy.stats import norm

SHOW = False

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CHAIN_DIR = os.path.join(DIR, 'chain_siphon')
ALL = []
RT = 'output'
ROOT = os.path.join(CHAIN_DIR, RT)

CONF_INT = -1  # either the percent confidence interval, or negative number for # of stdev to show
EXCLUDED = [
    'max',
    # 'control',
    # 'chain',
    # 'LJP',
    'leader',
]

OUTPUT_DIR = os.path.join(ROOT, 'plots')

VALUES = range(1, 31)
ROUND = 1
CARE = 1  # use numbers in linear reg if >CARE

for (plotting, PROP) in (('failed', False), ('failed', True), ('successful', False), ('successful', True)):
    legend_entries=0
    labeled_bars = False
    if plotting == 'failed':
        plotting = 'failed'
        y_ax = 'stuck'
    else:
        plotting = 'successful'
        y_ax = 'successful'
    if 'max' not in EXCLUDED:
        plt.plot(VALUES, [(1 if PROP else i) for i in VALUES], '-', alpha=.5, color='green', label='y = x')
        legend_entries+=1
    for folder in os.listdir(ROOT):
        if folder in EXCLUDED:
            continue
        fields = None
        data = []
        folder_dir = os.path.join(ROOT, folder)
        csv_files = [f for f in os.listdir(folder_dir) if f.endswith('.csv')]
        if not csv_files:
            continue
        # print(folder)
        for f in csv_files:
            # print('using:', f)
            fn = os.path.join(folder_dir, f)
            csvfile = open(fn)
            fields = None
            spamreader = csv.reader(csvfile)
            for row in spamreader:
                if fields is None:
                    fields = row
                else:
                    data.append(row)
            csvfile.close()

        data_dict = defaultdict(lambda: [])
        for row in data:
            data_dict[int(float(row[0]))].append(float(row[1]))
        # print(data_dict)
        keys = list(data_dict.keys())
        keys.sort()
        for entry in keys:
            arr = data_dict[entry]
            data_dict[entry] = {'successful': np.mean(arr),
                                'failed': entry - np.mean(arr),
                                'var': np.var(arr),
                                'number': len(arr)}
        y = np.array([data_dict[entry][plotting] for entry in keys])
        if PROP:
            y = y/keys
        var = np.array([data_dict[entry]['var'] for entry in keys])
        trials = np.array([data_dict[entry]['number'] for entry in keys])
        stdev = np.sqrt(var)
        # VALUES=DATA['num_agents']

        plt.plot(keys, y, alpha=1, label=folder)
        legend_entries+=1
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
            plt.fill_between(keys, low, high, alpha=.35, color='gray',
                             label=bar_label
                             )
            if bar_label is not None:
                legend_entries+=1
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
            plt.fill_between(keys, low, high, alpha=.35, color='gray',
                             label=bar_label
                             )
            if bar_label is not None:
                legend_entries+=1
        if False:
            XY = []
            for x, s in zip(range(1, len(y) + 1), y):
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
            legend_entries+=1
            # leg.append(('y = {0}*x' + (' + ' if b > 0 else ' ') + '{1}').format(round(m, ROUND), round(b, ROUND)))

    plt.xlabel('number of agents')

    plt.ylabel(('prop of ' if PROP else '') + y_ax + ' blimps')
    plt.ylim((0, 1.1 if PROP else plt.ylim()[1]))
    plt.legend(  # + leg_fill[0:1],
        # loc=('lower right' if PROP else 'upper left')
        # loc='center right', bbox_to_anchor=(1.2, .5),ncol=3,
        loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=legend_entries//2
    )
    save_file = os.path.join(OUTPUT_DIR, ('prop_' if PROP else '') + y_ax + '_rate.png')
    if not os.path.exists(os.path.dirname(save_file)):
        os.makedirs(os.path.dirname(save_file))
    plt.savefig(save_file)
    if SHOW:
        plt.show()
    plt.close()
