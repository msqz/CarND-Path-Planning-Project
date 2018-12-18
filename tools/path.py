#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

path = '../data/highway_map.csv'
map_data = []

with open(path) as f:
    for line in f.readlines():
        map_data.append([float(val) for val in line.split(' ')])

map_data = np.array(map_data)

path = '../stdout.log'
log_data = []
with open(path) as f:
    row = []
    for line in f.readlines():
        trimmed = line.strip()
        if trimmed.startswith('next_') and trimmed.endswith(']'):
            splitted = trimmed.replace('next_', '').replace(
                ':', '').replace('[', '').replace(']', '').split(' ')
            vals = [float(v) for v in splitted[1].split(',')]

            if splitted[0] == 's':
                row.append(vals)
            elif splitted[0] == 'd':
                row.append(vals)
            elif splitted[0] == 'x':
                row.append(vals)
            elif splitted[0] == 'y':
                row.append(vals)

        if len(row) == 4:    
            log_data.append(row)
            row = []

log_data = np.array(log_data)

waypoints = np.array(map_data)

for h in range(len(log_data)):

    plt.scatter(log_data[h, 2], log_data[h, 3])
    min_x = min(log_data[h, 2]) - 5
    max_x = max(log_data[h, 2]) + 5
    min_y = min(log_data[h, 3]) - 10
    max_y = max(log_data[h, 3]) + 10

    for i in range(len(map_data)):
        if min_x <= map_data[i, 0] <= max_x and min_y <= map_data[i, 1] <= max_y:
            x_real = [map_data[i, 0], map_data[i, 0] + map_data[i, 3] * 8]
            y_real = [map_data[i, 1], map_data[i, 1] + map_data[i, 4] * 8]
            plt.plot(x_real, y_real, color='orange', linestyle='--')
            x_used = [map_data[i, 0], map_data[i, 0]]
            y_used = [map_data[i, 1], map_data[i, 1] - 8]
            plt.plot(x_used, y_used, color='red', linestyle='--')

            plt.scatter(map_data[i,0], map_data[i,1])
    plt.show()
