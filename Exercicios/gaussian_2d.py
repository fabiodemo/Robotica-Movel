import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import norm, multivariate_normal

x, y = np.mgrid[-3:3:.01, -3:3:.01]
pos = np.dstack((x, y))
mv_norm = multivariate_normal([0, 0], [[1.0, 0], [0, 1.0]])

# Contour
fig = plt.figure(figsize=(8,15), dpi=100)
ax = fig.add_subplot(111, aspect='equal')
ax.contour(x, y, mv_norm.pdf(pos))
ax.axis([-3, 3, -3, 3])
plt.show()

# Surface
fig = plt.figure(figsize=(8,15), dpi=100)
ax = fig.add_subplot(111, projection='3d')

ax.plot_surface(x, y, mv_norm.pdf(pos), cmap='viridis', linewidth=0)
ax.set_xlabel('X')
ax.axis([-3, 3, -3, 3])
ax.view_init(elev=60, azim=90)
plt.show()