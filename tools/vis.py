#!/usr/bin/env python3
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json


def read(path):
  with open(path) as f:
    record = {}
    while True:
      line = f.readline()
      if line:
        sanitized = line.strip().replace(',}','}').replace(',]',']')
        import pdb;pdb.set_trace()
        record = json.loads(sanitized)
        yield record

def update(data):
  plots = []

  for p in range(2):
    start = [data[0], data[0] + 4]
    end = [data[1]-2, data[1]-2 + 4]
    plots.append(ax.plot(start, end))

  plots.append(ax.scatter([data[0]], [data[1]]))

  ax.set_xlim(data[0] - 5, data[0] + 5)
  ax.set_ylim(data[1] - 5, data[1] + 5)
  return plots


fig, ax = plt.subplots()

anim = FuncAnimation(fig, update, frames=read('../stdout.log'), interval=500)

plt.show()
