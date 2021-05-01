"""
launch.py

Launch the simulation for a specific map/configuration couple.
Usage: python ./launch.py <mapID> <configID>
"""

import csv, subprocess, time, os, signal, sys

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

                if mapId >= 2:
                    print('Reached mapId: {}'.format(mapId))
                    print('Exiting')
                    exit(0)

                yield 0, configId, mapId, start, goal

        except Exception as e:
            print(e)

def main(expectedMapId, expectedConfigId):
    for trialId, configId, mapId, start, goal in trialGenerator():
        if configId != expectedConfigId or mapId != expectedMapId:
            continue

        proc = subprocess.Popen(['roslaunch', 'autopilot', 'simulation.launch', #'monitoring:=true',
                                'x:={}'.format(start[0]), 
                                'y:={}'.format(start[1]), 
                                'z:={}'.format(start[2]), 
                                'goal_x:={}'.format(goal[0]), 
                                'goal_y:={}'.format(goal[1]), 
                                'goal_z:={}'.format(goal[2]), 
                                'world:=forest{}'.format(mapId), 
                                'trialId:={}'.format(trialId),
                                'configId:={}'.format(configId),
                                'mapId:={}'.format(mapId)])
        pid = proc.pid

        print('\n'*4)
        print('*'*30)
        print('Pipeline launched for trialId: {} | configId: {} | mapId: {}'.format(trialId, configId, mapId))
        print('*'*30, end='\n'*5)

        start = time.time()
        while True:
            time.sleep(5)

        print('\n\n\n*********** TIMEOUT ***********\n\n\n')
        print('Killing roslaunch...')
        os.kill(pid, signal.SIGINT)
        time.sleep(15)
        return


if __name__ == '__main__':
    mapId = int(sys.argv[1])
    configId = int(sys.argv[2])
    main(mapId, configId)