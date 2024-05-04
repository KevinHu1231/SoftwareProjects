import numpy as np
from ibvs_controller import ibvs_controller
from ibvs_simulation import ibvs_simulation
from dcm_from_rpy import dcm_from_rpy
import matplotlib.pyplot as plt
import time
# Camera intrinsics matrix - known.
def unknown_depth():
    K = np.array([[500.0, 0, 400.0], 
                [0, 500.0, 300.0], 
                [0,     0,     1]])

    # Target points (in target/object frame).
    pts = np.array([[-0.75,  0.75, -0.75,  0.75],
                    [-0.50, -0.50,  0.50,  0.50],
                    [ 0.00,  0.00,  0.00,  0.00]])

    # Camera poses, last and first.
    C_last = np.eye(3)
    t_last = np.array([[ 0.0, 0.0, -4.0]]).T
    # C_init = dcm_from_rpy([-np.pi/7, np.pi/5, -np.pi/6])
    # t_init = np.array([[0.3, -0.5, 3.1]]).T
    C_init = dcm_from_rpy([np.pi/9, -np.pi/7, -np.pi/8])
    t_init = np.array([[-0.4, 0.1, -4.3]]).T

    Twc_last = np.eye(4)
    Twc_last[0:3, :] = np.hstack((C_last, t_last))
    Twc_init = np.eye(4)
    Twc_init[0:3, :] = np.hstack((C_init, t_init))

    #gain = 0.1

    # Sanity check the controller output if desired.
    # ...

    # Run simulation - estimate depths.
    gains = []
    times = []
    i = 1
    while True:
        try:
            print("Iteration: " + str(i))
            gain = 0.05*i
            start = time.time()
            ibvs_simulation(Twc_init, Twc_last, pts, K, gain, False, False)
            end = time.time()
            conv_time = end - start
            gains += [gain]
            times += [conv_time]
            i+=1
        except:
            break

    plt.figure(2)
    plt.plot(gains,times)
    plt.grid(True)
    plt.title("Controller Gain and Corresponding Convergence Time with Unknown Depths")
    plt.xlabel("Gain")
    plt.ylabel("Convergence Time (s)")
    plt.show()

    return gains, times