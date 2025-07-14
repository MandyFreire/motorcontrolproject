import numpy as np
from scipy import signal

num = [0.01]
den = [0.005, 0.06, 0.1001]
system = signal.TransferFunction(num, den)

Kp_range = np.linspace(0.1, 2, 5)
Ki_range = np.linspace(0.01, 0.5, 5)
Kd_range = np.linspace(0, 0.2, 3)

best_overshoot = np.inf
best_params = None

for Kp in Kp_range:
    for Ki in Ki_range:
        for Kd in Kd_range:
            pid_num = [Kd, Kp, Ki]
            pid_den = [1, 0]
            pid = signal.TransferFunction(pid_num, pid_den)
            closed_loop = signal.feedback(pid * system, 1)
            t, y = signal.step(closed_loop, T=np.linspace(0, 10, 1000))
            overshoot = (np.max(y) - y[-1]) / y[-1] * 100

            if overshoot < best_overshoot:
                best_overshoot = overshoot
                best_params = (Kp, Ki, Kd)

print(f'Melhores parâmetros: Kp={best_params[0]}, Ki={best_params[1]}, Kd={best_params[2]}')
