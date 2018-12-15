#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

path = './stdout.log'
data = [[],[]]
with open(path) as f:
  for line in f.readlines():
    trimmed = line.strip()
    if trimmed.startswith('next_') and trimmed.endswith(']'):
      splitted = trimmed.replace('next_','').replace(':','').replace('[','').replace(']','').split(' ')
      vals = [float(v) for v in splitted[1].split(',')]
      if splitted[0] == 'x':
        data[0].append(vals)
      else:
        data[1].append(vals)

length = min(len(data[0]), len(data[1]))

plt.plot(data[0][:length], data[1][:length])
plt.show()