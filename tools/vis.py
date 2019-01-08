#!/usr/bin/env python3
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import json
import sys
import pdb


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
    plots.append(ax.plot(s_l, d_l))
    plots.append(ax.plot(s_r, d_r))

  s = np.array(data['next_s'])
  d = np.array(data['next_d'])
  
  s_l = s[:]
  s[0] -= 2.25
  s_r = s[:]
  s_r[-1] += 2.25
  d_l = d - 1.25
  d_r = d + 1.25
  plots.append(ax.plot(s_l, d_l))
  plots.append(ax.plot(s_r, d_r))
  plt.annotate('me', xy=(s_l[1],d_l[1]))

  x_lim_left = min([p['s'] for p in data['predictions']])
  x_lim_right = max([p['s'] for p in data['predictions']])
  
  ax.set_xlim(x_lim_left, x_lim_right)
  ax.set_ylim(14,-2)
  return plots


fig, ax = plt.subplots()
update(next(read(sys.argv[1])))
# anim = FuncAnimation(fig, update, frames=read(sys.argv[1]), interval=500)
#plt.gca().invert_yaxis()
plt.show()
