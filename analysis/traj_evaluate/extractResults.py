import csv, time, os, signal, json, numpy as np, matplotlib.pyplot as plt

N_TRIAL = 10
N_MAPS = 10

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

                if mapId > N_MAPS - 1:
                    raise StopIteration

                for i in range(N_TRIAL):
                    yield i, configId, mapId, start, goal

        except Exception as e:
            print(e)

def main():
    success = np.zeros(100 * N_MAPS)
    failureLocal = np.zeros(100 * N_MAPS)
    failureGlobal = np.zeros(100 * N_MAPS)
    failureMap = np.zeros(100 * N_MAPS)
    invalid = np.zeros(100 * N_MAPS)

    for trialId, configId, mapId, start, goal in trialGenerator():
        filename = 'test_m{}_c{}_t{}.json'.format(mapId, configId, trialId)
        path = '/home/rhidra/research_data/{}'.format(filename)

        i = mapId * 100 + configId

        if not os.path.exists(path):
            invalid[i] += 1
            continue
        
        try:
            with open(path, 'r') as f:
                data = json.load(f)
                if data['success']:
                    success[i] += 1
                elif data['failureType'] == 'local_planner':
                    failureLocal[i] += 1
                elif data['failureType'] == 'global_planner':
                    failureGlobal[i] += 1
                elif data['failureType'] == 'invalid_map':
                    failureMap[i] += 1
                else:
                    invalid[i] += 1
                print('\rAnalysing: [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId), end='')
        except Exception as e:
            print('\r', end='')
            print('*'*10)
            print('Error:     [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId))
            print(e)
            print('*'*10)
            invalid[i] += 1
            continue
    np.save('save6.npy', (success, failureLocal, failureGlobal, failureMap, invalid))
    print()


if __name__ == '__main__':
    main()