import numpy as np
import matplotlib.pyplot as plt

from scipy.stats import norm, multivariate_normal

x = np.linspace(-3, 3, 1000)

mu1 = 0
sigma21 = .15
y1 = norm.pdf(x, mu1, np.sqrt(sigma21))

mu2 = 2
sigma22 = .25
y2 = norm.pdf(x, mu2, np.sqrt(sigma22))

fig = plt.figure(figsize=(8,15), dpi=100)
ax = fig.add_subplot(111, aspect='equal')

ax.plot(x, y1, 'r-', label='y1')
ax.plot(x, y2, 'b-', label='y2')

ax.axis([-2, 4, 0, 2])
ax.grid()

mu_hat = (mu1 * sigma22 + mu2 * sigma21) / (sigma21 + sigma22)
sigma2_hat = 1 / (1/sigma21 + 1/sigma22)
y_hat = norm.pdf(x, mu_hat, np.sqrt(sigma2_hat))

ax.plot(x, y_hat, 'g-', label='y_hat')
plt.show()
