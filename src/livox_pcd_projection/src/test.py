from json import load
import numpy as np
import torch


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


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

import time

start = time.time()
mat1 = torch.randn(2, 3).to(device)
mat2 = torch.randn(3, 3).to(device)
m3 = torch.mm(mat1, mat2)
download = m3.cput().numpy()
end = time.time()
print(download, end - start)