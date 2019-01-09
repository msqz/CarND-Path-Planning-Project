#!/usr/bin/env python3
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import json
import sys
import pdb

def get_color():
  color_idx = -1
  colors = ['black','gray','navy','r','g','b','y','m','c'];
  while True:
    color_idx = (color_idx + 1) % (len(colors) - 1)
    yield colors[color_idx]

def read(path):
  with open(path) as f:
    record = {}
    while True:
      line = f.readline().strip()
      if line and line[-1] == '}':
        sanitized = line.replace(',}','}').replace(',]',']')
#        pdb.set_trace()
        record = json.loads(sanitized)
        yield record

def update(data):
  color_generator = get_color()
  plots = []

  for p in data['predictions']:
    s_l = [p['s_orig']-2.25, p['s']+2.25]
    s_r = [p['s_orig']-2.25, p['s']+2.25]
    d_l = [p['d_orig']-1.25, p['d']-1.25]
    d_r = [p['d_orig']+1.25, p['d']+1.25]

    label = '{}'.format(p['id'])
    for t in data['transitions']:
      for c in t['evaluation']['crash']:
        if c['id'] == p['id']:
          label += ' {}'.format(t['transition'])

    plt.annotate(label, xy=(s_l[1],d_l[1]))
    color = next(color_generator)
    plots.append(ax.plot(s_l, d_l, color=color))
    plots.append(ax.plot(s_r, d_r, color=color))

  if len(sys.argv) > 2:
    for t in data['transitions']:
      if t['transition'] == sys.argv[2]:
        s = np.array(t['path_s'])
        d = np.array(t['path_d'])
  else:
    s = np.array(data['next_s'])
    d = np.array(data['next_d'])
  
  s_l = s[:]
  s[0] -= 2.25
  s_r = s[:]
  s_r[-1] += 2.25
  d_l = d - 1.25
  d_r = d + 1.25
  color = next(color_generator)
  plots.append(ax.plot(s_l, d_l, color=color))
  plots.append(ax.plot(s_r, d_r, color=color))
  plt.annotate('me', xy=(s_l[1],d_l[1]))

  x_lim_left = min([p['s'] for p in data['predictions']])
  x_lim_right = max([p['s'] for p in data['predictions']])
  
  ax.set_xlim(x_lim_left-100, x_lim_right + 100)
  ax.set_ylim(14,-2)
  return plots


fig, ax = plt.subplots()
update(next(read(sys.argv[1])))
# anim = FuncAnimation(fig, update, frames=read(sys.argv[1]), interval=500)
#plt.gca().invert_yaxis()
plt.show()
