import numpy as np
import matplotlib.pyplot as plt

def gaussian_1d(x, mu, sigma):
    return (1/(sigma*np.sqrt(2*np.pi)))*np.exp(-0.5*((x-mu)/sigma)**2)

x = np.linspace(-2, 2, 100)

mu = 0
sigma = .2
y = gaussian_1d(x, mu, sigma)

fig = plt.figure(figsize=(15,15), dpi=400)
ax = fig.add_subplot(111, aspect='equal')

ax.plot(x, y, 'b-')

# ax.axis([-2, 2, 0, 1])
ax.grid(True)
plt.show()