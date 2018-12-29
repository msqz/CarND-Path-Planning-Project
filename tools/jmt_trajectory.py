#!/usr/bin/env python3
import matplotlib.pyplot as plt
import json

data_s = []
data_d = []

with open('./stdout.log') as f:
    state_to_jmt = {}
    s_tmp = None
    d_tmp = None

    for line in f:
        splitted = line.strip().split(':')
        if splitted[0] == 'jmt_s':
            s_tmp = [float(s) for s in (json.loads(splitted[1]))]
        elif splitted[0] == 'jmt_d':
            d_tmp = [-1 * float(d) for d in (json.loads(splitted[1]))]
        elif splitted[0] == 'transition' and ('RIGHT' in splitted[1] or 'STRAIGHT' in splitted[1]):
            state_to_jmt[splitted[1].strip()] = [s_tmp, d_tmp]
        elif splitted[0] == 'to' and splitted[1].strip() in state_to_jmt:
            data_s.append(state_to_jmt[splitted[1].strip()][0])
            data_d.append(state_to_jmt[splitted[1].strip()][1])
            s_tmp = None
            d_tmp = None
            state_to_jmt = {}


t = range(len(data_s[0]))

rows = 8
cols = (len(data_s)//8 + 1) * 2
fig, axes = plt.subplots(rows, cols)
for i, _ in enumerate(data_s):
    row = i % 8
    col_s = i//8 * 2
    col_d = col_s + 1
    print(row)
    print(col_s)
    print(col_d)
    print(i)
    axes[row, col_s].plot(t, data_s[i], color='blue')
    axes[row, col_d].plot(t, data_d[i], color='orange')
    print(data_s[i])
    print()
    print(data_d[i])
    print('\n\n')

plt.show()
