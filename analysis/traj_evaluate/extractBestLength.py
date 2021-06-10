import csv, time, os, signal, json, numpy as np, matplotlib.pyplot as plt

N_TRIAL = 1
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
    lengths = np.zeros(100 * N_MAPS)

    for trialId, configId, mapId, start, goal in trialGenerator():
        filename = 'best_m{}_c{}_t{}.json'.format(mapId, configId, trialId)
        path = '/home/rhidra/research_data/{}'.format(filename)

        i = mapId * 100 + configId

        if not os.path.exists(path):
            continue
        
        try:
            with open(path, 'r') as f:
                value = float(f.read())
                lengths[i] = value
                print('\rAnalysing: [trialId: {:3} | configId: {:3} | mapId: {:3}] ({:2.3})'.format(trialId, configId, mapId, value), end='')
        except Exception as e:
            print('\r', end='')
            print('*'*10)
            print('Error:     [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId))
            print(e)
            print('*'*10)
            continue
    np.save('bestLength.npy', lengths)
    print()


if __name__ == '__main__':
    main()