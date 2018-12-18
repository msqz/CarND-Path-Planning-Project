#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

path = '../data/highway_map.csv'
data = []

with open(path) as f:
  for line in f.readlines():
    data.append([float(val) for val in line.split(' ')])

data = np.array(data)

fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
ax = fig.add_subplot(111)
# ax.scatter(data[:,0],data[:,1], data[:,2])
ax.scatter(data[:,0],data[:,1])
for i in range(len(data)):
  ax.plot([data[i, 0], data[i, 0] + data[i, 3]*100], [data[i, 1], data[i, 1]+data[i, 4]*100])
# ax.plot(data[:,0]+data[:,3], data[:,1]+data[:,4], data[:,2])
plt.show()