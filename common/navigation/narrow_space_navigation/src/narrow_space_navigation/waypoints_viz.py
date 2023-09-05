#!/usr/bin/env python3
# python3 -m pip install plotly numpy
# python3 grid.py

# import plotly.graph_objects as go
import matplotlib.pyplot as plt
import numpy as np
import math

# initialise empty heightmap
SIZE = 50
heights = np.zeros([SIZE, SIZE])

# define points where we think people are standing
standing = [
    (12, 25),
    (38, 17)
]

# standard deviation
def std(dist, impact, spread):
    v1 = 1 / (impact * math.sqrt(2 * math.pi))
    v2 = math.pow(dist, 2) / (2 * math.pow(spread, 2))
    return v1 * math.pow(math.e, -v2)

# apply a crude normal distribution over the entire heightmap from points
IMPACT = 2
SPREAD = 8

for (x, y) in standing:
    for targetX in range(0, SIZE):
        if targetX >= 0 and targetX < SIZE:
            for targetY in range(0, SIZE):
                if targetY >= 0 and targetY < SIZE:
                    dist = math.sqrt((targetX - x) ** 2 + (targetY - y) ** 2)
                    heights[targetX][targetY] += std(dist, IMPACT, SPREAD)

# do the same from the edges
IMPACT = 4
SPREAD = 6

for x in range(0, SIZE):
    for y in range(0, SIZE):
        heights[x][y] += std(x, IMPACT, SPREAD) \
            + std(y, IMPACT, SPREAD) \
            + std(SIZE - x, IMPACT, SPREAD) \
            + std(SIZE - y, IMPACT, SPREAD)

# convert to points
points = np.empty([SIZE * SIZE, 3])
for x in range(0, SIZE):
    for y in range(0, SIZE):
        points[y * SIZE + x][0] = x
        points[y * SIZE + x][1] = y
        points[y * SIZE + x][2] = heights[x][y]

# mark the points where people are standing for clarity
for (x, y) in standing:
    heights[x][y] = 1

# iterate through height map and find least busy point
h = math.inf
p = (0, 0)
for x in range(0, SIZE):
    for y in range(0, SIZE):
        c = heights[x][y]
        if c < h:
            h = c
            p = (x, y)

print('Best point is', p)


# Create a 3D scatter plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=3, alpha=0.8, c='b', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()