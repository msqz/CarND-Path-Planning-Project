#!/usr/bin/env python3
import matplotlib.pyplot as plt
import sys
import json

DELTA_T = 0.02
PATH_LENGTH = 100

data_s = json.loads(next(sys.stdin))
data_d = json.loads(next(sys.stdin))
plt.scatter([i * 0.02 for i in range(0, PATH_LENGTH)], data_s)
plt.scatter([i * 0.02 for i in range(0, PATH_LENGTH)], data_d)
plt.show()
