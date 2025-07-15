import time
import numpy as np

# Simulação simplificada
def collect_data():
    data = []
    for t in range(1000):
        setpoint = 100  # rpm desejada
        current_speed = np.random.normal(95, 5)  # simula leitura real do motor
        error = setpoint - current_speed
        control_signal = 0.1 * error  # pid simplificado

        data.append([setpoint, current_speed, control_signal, error])
        time.sleep(0.01)
    np.save("motor_data.npy", np.array(data))

if __name__ == "__main__":
    collect_data()
