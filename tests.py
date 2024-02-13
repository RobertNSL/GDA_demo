import numpy as np
import matplotlib.pyplot as plt


x = []
y = []
for r in np.arange(5, 1, -1):
    for theta in np.arange(0, 360, 0.1):
        x.append(r*np.cos(np.deg2rad(theta)))
        y.append(r*np.sin(np.deg2rad(theta)))

plt.plot(x, y)
plt.show()
