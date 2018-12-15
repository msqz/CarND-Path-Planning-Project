#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

path = './data/highway_map.csv'
data = []

with open(path) as f:
  for line in f.readlines():
    data.append([float(val) for val in line.split(' ')])

data = np.array(data)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(data[:,0],data[:,1], data[:,2])
ax.plot(data[:,0]+data[:,3], data[:,1]+data[:,4], data[:,2])
plt.show()