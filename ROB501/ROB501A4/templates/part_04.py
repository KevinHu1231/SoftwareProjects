import numpy as np
from part_02_learner_example_ import known_depth
from part_03_learner_example_ import unknown_depth
import matplotlib.pyplot as plt

gains_k, times_k = known_depth()
gains_u, times_u = unknown_depth()
k_op = np.array(gains_k)[np.argmin(np.array(gains_k))]
u_op = np.array(gains_u)[np.argmin(np.array(gains_u))]

print(k_op)
print(u_op)

time_diff = np.subtract(np.array(times_k),np.array(times_u))

plt.figure(3)
plt.plot(gains_k[2:-4],time_diff[2:-4])
plt.grid(True)
plt.title("Controller Gain and Corresponding Difference in Convergence Time Between Known and Unknown Depths")
plt.xlabel("Gain")
plt.ylabel("Convergence Time (s)")
plt.show()
