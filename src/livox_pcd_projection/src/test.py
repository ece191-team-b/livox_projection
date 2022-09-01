from json import load
import numpy as np



def load_intrinsic_distortion(intrinsic_path):
        file = open(intrinsic_path, 'r')
        lines = file.readlines()
        intrinsic = np.zeros((3, 3))
        distortion = np.zeros((5, 1))
        skip = [0, 4, 5]
        for i, line in enumerate(lines):
            if i in skip:
                continue
            elif i < 3:
                line = line.split(' ')
                print(line)
                intrinsic[i-1]= [float(x) for x in line if x != '']
            else:
                line = line.split(' ')
                for j, dist in enumerate(line):
                    if dist != '':
                        distortion[j] = float(dist)
        print(intrinsic)
        print(distortion)
        file.close()

load_intrinsic_distortion('/home/chengjing/Desktop/livox_projection/calibration_data/parameters/intrinsic.txt')