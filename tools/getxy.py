#!/usr/bin/env python3

import matplotlib.pyplot as plt
import math
import numpy as np

path = '../data/highway_map.csv'
data = []

with open(path) as f:
    for line in f.readlines():
        data.append([float(val) for val in line.split(' ')])

data = np.array(data)


# def closestWaypoint(x, y, maps_x, maps_y):
#   closestLen = 100000;  # large number
#   closestWaypoint = 0

#   for i in range(len(maps_x)):
#     map_x = maps_x[i]
#     map_y = maps_y[i]
#     dist = distance(x, y, map_x, map_y)
#     if dist < closestLen:
#       closestLen = dist
#       closestWaypoint = i

#   return closestWaypoint


# def nextWaypoint(x, y, theta, maps_x, maps_y):
#   closestWaypoint = closestWaypoint(x, y, maps_x, maps_y)

#   map_x = maps_x[closestWaypoint]
#   map_y = maps_y[closestWaypoint]

#   heading = math.atan2((map_y - y), (map_x - x))

#   angle = abs(theta - heading)
#   angle = min(2 * pi() - angle, angle)

#   if angle > pi() / 4:
#     closestWaypoint = closestWaypoint + 1
#     if closestWaypoint == maps_x.size():
#       closestWaypoint = 0

#   return closestWaypoint


# }

# def distance(x1, y1, x2, y2):
#   return math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))

# # Transform from Cartesian x,y coordinates to Frenet s,d coordinates
# def getFrenet(x, y, theta, maps_x, maps_y):
#   next_wp = nextWaypoint(x, y, theta, maps_x, maps_y)

#   prev_wp
#   prev_wp = next_wp - 1
#   if next_wp == 0:
#     prev_wp = len(maps_x) - 1

#   n_x = maps_x[next_wp] - maps_x[prev_wp]
#   n_y = maps_y[next_wp] - maps_y[prev_wp]
#   x_x = x - maps_x[prev_wp]
#   x_y = y - maps_y[prev_wp]

#   # find the projection of x onto n
#   proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y)
#   proj_x = proj_norm * n_x
#   proj_y = proj_norm * n_y

#   frenet_d = distance(x_x, x_y, proj_x, proj_y)

#   # see if d value is positive or negative by comparing it to a center point

#   center_x = 1000 - maps_x[prev_wp]
#   center_y = 2000 - maps_y[prev_wp]
#   centerToPos = distance(center_x, center_y, x_x, x_y)
#   centerToRef = distance(center_x, center_y, proj_x, proj_y)

#   if centerToPos <= centerToRef:
#     frenet_d = frenet_d * -1

#   # calculate s value
#   frenet_s = 0
#   for i in range(prev_wp):
#     frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1])

#   frenet_s = frenet_s + distance(0, 0, proj_x, proj_y)

#   return (frenet_s, frenet_d)

def getXY(s, d, maps_s, maps_x, maps_y, maps_dx, maps_dy):
    waypoints_normalized = []
    for i in range(len(maps_s)):
        preceeding_wp = i - 1
        if preceeding_wp < 0:
            preceeding_wp = len(maps_s) - abs(preceeding_wp)
        
        angle_prev = math.atan2(maps_dy[preceeding_wp], maps_dx[preceeding_wp])
        angle_current = math.atan2(maps_dy[i], maps_dx[i])
        angle = abs(angle_prev - angle_current) / 2.0
        norm = (angle/2*math.pi) * (2.0 * math.pi * d); 
        waypoint_s_norm = maps_s[i] - norm

        waypoints_normalized.append(waypoint_s_norm)

    prev_wp = -1
    while s > waypoints_normalized[prev_wp + 1] and prev_wp < (len( waypoints_normalized) - 1):
        prev_wp = prev_wp + 1

    wp2 = (prev_wp + 1) % len(maps_x)
    print('{} to {}'.format(prev_wp, wp2))
    dist = (maps_y[wp2] - maps_y[prev_wp], maps_x[wp2] - maps_x[prev_wp])
    heading = math.atan2(dist[0], dist[1])
    print('heading: {}'.format(heading))

    # the x,y,s along the segment
    # seg_s = (s - maps_s[prev_wp])
    seg_s = (s - waypoints_normalized[prev_wp])
    print('seg_s: {}'.format(seg_s))

    seg_x = maps_x[prev_wp] + seg_s * math.cos(heading)
    seg_y = maps_y[prev_wp] + seg_s * math.sin(heading)
    print('seg_x: {}'.format(seg_x))
    print('seg_y: {}'.format(seg_y))

    perp_heading = heading - math.pi / 2
    print('perp_heading: {}'.format(perp_heading))

    x = seg_x + d * math.cos(perp_heading)
    y = seg_y + d * math.sin(perp_heading)

    return x, y

# # 961.87, 1131.80
# print(getXY(176.77, 6.16, data[:, 2], data[:, 0], data[:, 1], data[:, 3], data[:, 4]))
# print('-----------------')
# # 963.08, 1131.60
# print(getXY(177.20, 6.16, data[:, 2], data[:, 0], data[:, 1], data[:, 3], data[:, 4]))


next_s = [148.2396895968,148.5172791936,148.7968687904,149.0784583872,149.362047984,149.6476375808,149.9352271776,150.2248167744,150.5164063712,150.809995968,151.1055855648,151.4031751616,151.7027647584,152.0043543552,152.307943952,152.6135335488,152.9211231456,153.2307127424,153.5423023392,153.855891936,154.1714815328,154.4890711296,154.8086607264,155.1302503232,155.45383992,155.7794295168,156.1070191136,156.4366087104,156.7681983072,157.101787904,157.4373775008,157.7749670976,158.1145566944,158.4561462912,158.799735888,159.1453254848,159.4929150816,159.8425046784,160.1940942752,160.547683872,160.9032734688,161.2608630656,161.6204526624,161.9820422592,162.345631856,162.7112214528,163.0788110496,163.4484006464,163.8199902432,164.19357984,164.5691694368,164.9467590336,165.3263486304,165.7079382272,166.091527824,166.4771174208,166.8647070176,167.2542966144,167.6458862112,168.039475808,168.4350654048,168.8326550016,169.2322445984,169.6338341952,170.037423792,170.4430133888,170.8506029856,171.2601925824,171.6717821792,172.085371776,172.5009613728,172.9185509696,173.3381405664,173.7597301632,174.18331976,174.6089093568,175.0364989536,175.4660885504,175.8976781472,176.331267744,176.7668573408,177.2044469376,177.6440365344,178.0856261312,178.529215728,178.9748053248,179.4223949216,179.8719845184,180.3235741152,180.777163712,181.2327533088,181.6903429056,182.1499325024,182.6115220992,183.075111696,183.5407012928,184.0082908896,184.4778804864,184.9494700832,185.42305968]
next_d = [6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812,6.164812]

next_x = []
next_y = []

for i in range(len(next_s)):
    x, y = getXY(next_s[i], next_d[i], data[:, 2], data[:, 0], data[:, 1], data[:, 3], data[:, 4])
    next_x.append(x)
    next_y.append(y)

plt.scatter(next_x, next_y)
min_x = min(next_x) - 5
max_x = max(next_x) + 5
min_y = min(next_y) - 10
max_y = max(next_y) + 10

for i in range(len(data)):
    if min_x <= data[i, 0] <= max_x and min_y <= data[i, 1] <= max_y:
        x_real = [data[i, 0], data[i, 0] + data[i, 3] * 8]
        y_real = [data[i, 1], data[i, 1] + data[i, 4] * 8]
        plt.plot(x_real, y_real, color='orange', linestyle='--')
        x_used = [data[i, 0], data[i, 0]]
        y_used = [data[i, 1], data[i, 1] - 8]
        plt.plot(x_used, y_used, color='red', linestyle='--')

        plt.scatter(data[i,0], data[i,1])
plt.show()