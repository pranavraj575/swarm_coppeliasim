import os,csv,numpy as np
from collections import defaultdict
def datadict_from_folder(folder):
    """
    returns a datadict object from the folder specified
    @param folder: should have files like folder/data1.csv, ...
    @return data_dict with keys being the agent number, with each key containing a dictionary
        {
            'successful': mean # of successful blimps
            'failed': mean # of failed blimps
            'var': variance of successful/failed blimps
            'trials': number of trials
        }
        also data, contianing all rows of the csv files
    """
    csv_files = [f for f in os.listdir(folder) if f.endswith('.csv')]
    if not csv_files:
        return None, None
    # print(folder)
    data = []
    for f in csv_files:
        # print('using:', f)
        fn = os.path.join(folder, f)
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
                            'trials': len(arr)}
    return data_dict, data