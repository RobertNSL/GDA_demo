import numpy as np
from scipy.signal import butter
import matplotlib.pyplot as plt



class ESC:
    def __init__(self):
        self.sample_freq = 100
        self.dt = 1 / self.sample_freq
        self.A = 0.2
        self.omega = 10 * 2 * np.pi  # 10 Hz
        self.phase = 0
        self.K = 5
        self.butterorder = 1
        self.butterfreq = 2  # in Hz for 'high'
        self.b, self.a = butter(self.butterorder, self.butterfreq*self.dt*2, 'high', True)


# Objective function definition
def J(u):
    return 25 - (5 - u)**2


u = 0
y0 = J(u)

# Extremum Seeking Control Parameters
freq = 10  # sample frequency
dt = 1 / freq
T = 20  # total period of simulation (in seconds)

# Perturbation parameters
A = 0.1  # amplitude
omega = 1 * 2 * np.pi
phase = 0
K = 2  # integration gain

# High pass filter
butterorder = 1
butterfreq = 2  # in Hz for 'high'
b, a = butter(butterorder, butterfreq * dt * 2, 'high')
ys = np.zeros(butterorder + 1) + y0
HPF = np.zeros(butterorder + 1)

uhat = u
time = np.arange(0, T, dt)
yvals = np.zeros_like(time)
uhats = np.zeros_like(time)
uvals = np.zeros_like(time)

for i, t in enumerate(time):
    yvals[i] = J(u)

    for k in range(butterorder):
        ys[k] = ys[k + 1]
        HPF[k] = HPF[k + 1]
    ys[butterorder] = yvals[i]

    HPFnew = 0
    for k in range(butterorder+1):
        HPFnew = HPFnew + b[k]*ys[butterorder-k]
    for k in range(1, butterorder+1):
        HPFnew = HPFnew - a[k] * HPF[butterorder + 1 - k]
    HPFnew /= a[0]
    HPF[butterorder] = HPFnew

    xi = HPFnew * np.sin(omega * t + phase)
    uhat += xi * K * dt
    u = uhat + A * np.sin(omega * t + phase)
    uhats[i] = uhat
    uvals[i] = u

# Plotting
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time, uvals, label='$u$', linewidth=1.2)
plt.plot(time, uhats, label='$\hat{u}$', linewidth=1.2)
plt.legend(loc='upper right')
plt.grid(True)
plt.subplot(2, 1, 2)
plt.plot(time, yvals, linewidth=1.2)
plt.ylim(-1, 26)
plt.grid(True)

plt.tight_layout()
plt.show()
