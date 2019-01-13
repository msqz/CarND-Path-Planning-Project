#!/usr/bin/env python3

import log;
import matplotlib.pyplot as plt

colors = log.get_color(4)
fig, ax = plt.subplots()

records = [r for r in log.read('stdout.log')]
print('Found {} records'.format(len(records)))

# print('   {:>20}  {:>20}'.format('current diff', 'prev diff'))
x_next_x_diff = 0
prev_x_next_x_diff = 0
y_next_y_diff = 0
prev_y_next_y_diff = 0
# for i in range(1, len(records)):
rng = range(52, 54)
for i in rng:

  # ax.scatter()
  color = next(colors)
  ax.scatter(records[i]['x'], records[i]['y'], color=color, marker='+')
  ax.scatter(records[i]['prev_x'][0], records[i]['prev_y'][0], color=color, marker='_')
  ax.plot(records[i]['next_x'], records[i]['next_y'], color=color)
  ax.annotate(records[i]['to'], xy=(records[i]['x'],records[i]['y']))

  x_next_x_diff += abs(records[i]['next_x'][0] - records[i]['x'])
  prev_x_next_x_diff += abs(records[i]['next_x'][0] - records[i]['prev_x'][0])
  y_next_y_diff += abs(records[i]['next_y'][0] - records[i]['y'])
  prev_y_next_y_diff += abs(records[i]['next_y'][0] - records[i]['prev_y'][0])
    
  # print('x: {:>20}  {:>20}'.format(x_next_x_diff, y_next_y_diff))
  # print('y: {:>20}  {:>20}'.format(prev_x_next_x_diff, prev_y_next_y_diff))
  # print('-'*34)

  # if x_last is not None:
    # print('{:>5}: {:>8.8f}, {:>8.8f}'.format('start', x_last, y_last))
  # ax.annotate('{:>5}: {:>8.8f}, {:>8.8f}'.format('start', x[0], y[0]), xy=(x[0],y[0]))
  # ax.scatter(x[0], y[0])
  # print('-'*25)

  # x_last = x[0]
  # y_last = y[0]

print('x diff: {}'.format(x_next_x_diff/len(rng)))
print('y diff: {}'.format(y_next_y_diff/len(rng)))
print('prev_x diff: {}'.format(prev_x_next_x_diff/len(rng)))
print('prev_y diff: {}'.format(prev_y_next_y_diff/len(rng)))

plt.show()