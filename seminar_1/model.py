import numpy as np
from scipy.integrate import odeint
import math
import matplotlib.pyplot as plt

g = 10
L = 0.5
q0 = 0.5 # starting position
dt = 1/240 # time step
t = 0
maxTime = 5
logTime = np.arange(0.0, maxTime, dt)
sz = logTime.size

def rp(x, t):
    return [
        x[1],
        -g/L*math.sin(x[0])
    ]

sol = odeint(rp, [q0, 0], logTime)
logTheta = sol[:, 0]
logOmega = sol[:, 1]

np.save(rf"seminar_1\result\array\01_theor_theta.npy", logTheta)
np.save(rf"seminar_1\result\array\02_theor_omega.npy", logOmega)

plt.grid()
plt.plot(logTime, logTheta, label = "theor_theta")
plt.legend()
plt.savefig(rf"seminar_1\result\plot\01_theor_theta.png")
plt.show()

plt.grid()
plt.plot(logTime, logOmega, label = "theor_omega")
plt.legend()
plt.savefig(rf"seminar_1\result\plot\02_theor_omega.png")
plt.show()