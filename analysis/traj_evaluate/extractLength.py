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
    lengths = [[] for _ in range(100 * N_MAPS)]

    for trialId, configId, mapId, start, goal in trialGenerator():
        filename = 'test_m{}_c{}_t{}.json'.format(mapId, configId, trialId)
        path = '/home/rhidra/research_data6/{}'.format(filename)

        i = mapId * 100 + configId

        if not os.path.exists(path):
            continue
        
        try:
            with open(path, 'r') as f:
                data = json.load(f)
                if not data['success']:
                    continue
                x = np.array(data['uav_pos_x'])
                y = np.array(data['uav_pos_y'])
                z = np.array(data['uav_pos_z'])
                pos = np.vstack((x, y, z)).T
                total = 0
                for j in range(pos.shape[0] - 1):
                    total += np.linalg.norm(pos[j] - pos[j + 1])
                lengths[i].append(total)
                print('\rAnalysing: [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId), end='')
        except Exception as e:
            print('\r', end='')
            print('*'*10)
            print('Error:     [trialId: {:3} | configId: {:3} | mapId: {:3}]'.format(trialId, configId, mapId))
            print(e)
            print('*'*10)
            continue
    lengths = np.array([np.mean(t if len(t) > 0 else [0]) for t in lengths])
    np.save('length6.npy', lengths)
    print()


if __name__ == '__main__':
    main()