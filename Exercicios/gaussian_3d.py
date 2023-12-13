import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the parameters of the Gaussian distribution
mu = np.array([0, 0])  # Mean
cov = np.array([[1, 0], [0, 1]])  # Covariance matrix

# Generate grid points
x = np.linspace(-3, 3, 100)
y = np.linspace(-3, 3, 100)
X, Y = np.meshgrid(x, y)

# Calculate the Gaussian values at each grid point
Z = np.zeros_like(X)
for i in range(len(x)):
    for j in range(len(y)):
        point = np.array([X[i, j], Y[i, j]])
        Z[i, j] = 1 / (2 * np.pi * np.sqrt(np.linalg.det(cov))) * np.exp(-0.5 * np.dot(np.dot((point - mu), np.linalg.inv(cov)), (point - mu).T))

# Plot the 3D Gaussian distribution
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, cmap='viridis')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
