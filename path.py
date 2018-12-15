#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

path = './stdout.log'
data = [[],[], [], []]
with open(path) as f:
  for line in f.readlines():
    trimmed = line.strip()
    if trimmed.startswith('next_') and trimmed.endswith(']'):
      splitted = trimmed.replace('next_','').replace(':','').replace('[','').replace(']','').split(' ')
      vals = [float(v) for v in splitted[1].split(',')]
      if splitted[0] == 'x':
        data[0].append(vals)
      elif splitted[0] == 'y':
        data[1].append(vals)
      elif splitted[0] == 's':
        data[2].append(vals)
      elif splitted[0] == 'd':
        data[3].append(vals)

length = min(len(data[0]), len(data[1]))

print('Found {} paths'.format(length))

for i in range(length):
  plt.scatter(data[0][i], data[1][i])
  plt.show()