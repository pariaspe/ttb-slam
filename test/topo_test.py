#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import cv2


def consecutive(data, stepsize=1):
    if len(data) == 0:
        return [()]
    consec = np.split(data, np.where(np.diff(data) != stepsize)[0]+1)
    return [(a[0], a[-1]) for a in consec]


# explored_map = np.genfromtxt("../src/Generated/explored_map.csv", delimiter=',')
explored_map = np.genfromtxt("../robots_moviles/maps/map1/map1.csv", delimiter=',')
# explored_map = np.genfromtxt("../robots_moviles/maps/map2/map2.csv", delimiter=',')
# explored_map = np.genfromtxt("../robots_moviles/maps/map3/map3.csv", delimiter=',')
width, height = np.shape(explored_map)
print(explored_map)
plt.imshow(explored_map, interpolation='nearest')
plt.pause(3)

# Get rows of empty cells
hvertex = []
for i in range(width):
    free = np.where(explored_map[i, :] == 0.0)
    segments = consecutive(free[0])
    for seg in segments:
        if seg:
            idx = [(i, seg[0]), (i, seg[1])]
        else:
            continue
        hvertex.append(idx)
# Sort
hvertex = sorted(hvertex)
# Get biggest rect of rows
while True:
    last = len(hvertex)
    for i, rect in enumerate(hvertex):
        if i == 0:
            continue
        prev = hvertex[i-1]
        # if rect_diff(rect, prev):
        if rect[0][1] == prev[0][1] and rect[1][1] == prev[1][1]:
            hvertex.insert(i-1, [prev[0], rect[1]])
            hvertex.remove(prev)
            hvertex.remove(rect)
    if last == len(hvertex):
        break
print(hvertex)
print()

# Get columns of empty cells
vvertex = []
for j in range(height):
    free = np.where(explored_map[:, j] == 0.0)
    segments = consecutive(free[0])
    for seg in segments:
        if seg:
            idx = [(seg[0], j), (seg[1], j)]
        else:
            continue
        vvertex.append(idx)
# Sort
vvertex = sorted(vvertex)
# Get biggest rect in columns
while True:
    last = len(vvertex)
    for i, rect in enumerate(vvertex):
        if i == 0:
            continue
        prev = vvertex[i-1]
        # if rect_diff(rect, prev):
        if rect[0][0] == prev[0][0] and rect[1][0] == prev[1][0]:
            vvertex.insert(i-1, [prev[0], rect[1]])
            vvertex.remove(prev)
            vvertex.remove(rect)
    if last == len(vvertex):
        break
print(vvertex)


def intersection(a, b):
    x1 = max(min(a[0][0], a[1][0]), min(b[0][0], b[1][0]))
    y1 = max(min(a[1][0], a[1][1]), min(b[1][0], b[1][1]))
    x2 = min(max(a[0][0], a[1][0]), max(b[0][0], b[1][0]))
    y2 = min(max(a[1][0], a[1][1]), max(b[1][0], b[1][1]))
    if x1 < x2 and y1 < y2:
        return x1, y1, x2, y2


def intersection2(a, b):
    ax1 = a[0][0]
    ax2 = a[0][1]
    ay1 = a[1][0]
    ay2 = a[1][1]
    bx1 = b[0][0]
    bx2 = b[0][1]
    by1 = b[1][0]
    by2 = b[1][1]
    x1 = max(min(ax1, ax2), min(bx1, bx2))
    y1 = max(min(ay1, ay2), min(by1, by2))
    x2 = min(max(ax1, ax2), max(bx1, bx2))
    y2 = min(max(ay1, ay2), max(by1, by2))
    if a[1][0] < b[0][0] or b[1][0] < a[0][0] or a[1][1] < b[0][1] or b[1][1] < a[0][1]:
        return None
    return [(max(ax1, bx1), min(ax2, bx2)), (max(ay1, by1), min(ay2, by2))]
    # return [(x1, y1), (x2, y2)]


print(intersection2(hvertex[0], vvertex[0]))
print(intersection2(hvertex[0], vvertex[1]))
print(intersection2(hvertex[2], hvertex[0]))


def fill_rect(array, poses, value):
    a = np.copy(array)
    for i, row in enumerate(a):
        for j, e in enumerate(a):
            if poses[0][0] <= i <= poses[1][0] and poses[0][1] <= j <= poses[1][1]:
                a[i][j] += value
    return a


a = np.array(explored_map * 255, dtype=np.uint8).astype('uint8')
rects = hvertex + vvertex
v = 50
for rect in rects:
    a = fill_rect(a, rect, v)
    v += 15
print(a)

plt.imshow(a, cmap="Greys", interpolation='nearest')
plt.pause(0)
