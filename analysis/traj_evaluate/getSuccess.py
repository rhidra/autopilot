import csv, time, os, signal, json, numpy as np, matplotlib.pyplot as plt

N_TRIAL = 10

def trialGenerator():
    with open('/home/rhidra/forest_gen/start_and_end.csv', 'r') as f:
        try:
            reader = csv.reader(f)
            next(reader)
            for row in reader:
                configId = int(row[0]) % 100
                mapId = int(row[1])
                start = [float(row[2]), float(row[3]), float(row[4])]
                goal = [float(row[5]), float(row[6]), float(row[7])]

                for i in range(N_TRIAL):
                    yield i, configId, mapId, start, goal

        except Exception as e:
            print(e)

def main():
    success = np.zeros(100*2)
    failure = np.zeros(100*2)
    empty = np.zeros(100*2)

    for trialId, configId, mapId, start, goal in trialGenerator():
        filename = 'test_m{}_c{}_t{}.json'.format(mapId, configId, trialId)
        path = '/home/rhidra/research_data/{}'.format(filename)

        if not os.path.exists(path):
            # success[configId] = -1
            continue

        i = mapId * 100 + configId
        
        
        try:
            with open(path, 'r') as f:
                data = json.load(f)
                if data['success']:
                    success[i] += 1
                else:
                    failure[i] += 1
                print('\rAnalysing: [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId), end='')
        except Exception as e:
            print('\r', end='')
            print('*'*10)
            print('Error:     [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId))
            print(e)
            print('*'*10)
            empty[i] += 1
            # success[configId] = -1
            continue
    np.save('save2_success.npy', success)
    np.save('save2_failure.npy', failure)
    np.save('save2_empty.npy', empty)


if __name__ == '__main__':
    main()