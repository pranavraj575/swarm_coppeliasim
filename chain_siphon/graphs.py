import csv, os, sys
from matplotlib import pyplot as plt
import numpy as np
from scipy.stats import norm

DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))
CHAIN_DIR = os.path.join(DIR, 'chain_siphon')
ALL = []
RT = 'test_output'
RT = 'output_old_blimps_frictionless'
ROOT = os.path.join(CHAIN_DIR, RT)

OUTPUT_DIR = os.path.join(ROOT, 'plots')
SHOW = True
VALUES = range(1, 25)
ROUND = 1
CARE = 1  # use numbers in linear reg if >CARE
for PROP in (False, True):
    plt.plot(VALUES, [(1 if PROP else i) for i in VALUES], '-', alpha=.5, color='green')
    leg = []
    leg_fill = []
    leg.append('y = x')  # (theor. MAX)')

    for folder in os.listdir(ROOT):
        fields = None
        data = []

        fn = os.path.join(ROOT, folder, 'data.csv')
        if not os.path.exists(fn):
            continue
        csvfile = open(fn)
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            if fields is None:
                fields = row
            else:
                data.append(row)
        data.sort(key=lambda row: float(row[fields.index('num_agents')]))
        csvfile.close()
        DATA = dict()
        for i in range(len(fields)):
            DATA[fields[i]] = np.array([float(row[i]) for row in data])
        y = DATA['succ']/(DATA['num_agents'] if PROP else 1)
        var = DATA['variance'] if 'variance' in DATA else np.zeros(len(y))
        stdev = np.sqrt(var)
        CONF_INT = 95

        percentile = 1 - (1 - CONF_INT/100)/2
        # since double sided

        # 95% confidence interval, subtract 1 from num trials since 1 DOF lost for variance
        conf = (stdev/np.sqrt(DATA['trials'] - 1))*norm.ppf(percentile)
        if PROP:
            conf = conf/DATA['num_agents']

        # VALUES=DATA['num_agents']
        low = np.max((y - conf, [0 for _ in range(len(y))]), axis=0)
        high = np.min((y + conf, [(1 if PROP else i) for i in range(1, 1 + len(y))]), axis=0)

        plt.plot(DATA['num_agents'], y, alpha=1)
        plt.fill_between(DATA['num_agents'], low, high, alpha=.35, color='gray')
        leg.append(folder)
        leg_fill.append(str(CONF_INT) + '% conf. int')
        ALL.append(y)
        XY = []
        for x, s in zip(range(1, len(DATA['succ']) + 1), DATA['succ']):
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
                 alpha=.5, color='purple')
        leg.append(('y = {0}*x' + (' + ' if b > 0 else ' ') + '{1}').format(round(m, ROUND), round(b, ROUND)))

    plt.xlabel('number of agents')

    plt.ylabel(('prop of ' if PROP else '') + 'successful blimps')
    plt.ylim((0, 1.1 if PROP else plt.ylim()[1]))
    plt.legend(leg + leg_fill[0:1],
               # loc=('lower right' if PROP else 'upper left')
               # loc='center right', bbox_to_anchor=(1.2, .5),ncol=3,
               loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=3
               )
    save_file = os.path.join(OUTPUT_DIR, ('prop_' if PROP else '') + 'success_rate.png')
    if not os.path.exists(os.path.dirname(save_file)):
        os.makedirs(os.path.dirname(save_file))
    plt.savefig(save_file)
    if SHOW:
        plt.show()
    plt.close()
