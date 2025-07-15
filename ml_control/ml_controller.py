import numpy as np
from tensorflow.keras.models import load_model

model = load_model("trained_model.h5")

def ml_control(setpoint, speed):
    inputs = np.array([[setpoint, speed, 0]])  # 0 pode ser último controle ou outro input
    control_signal = model.predict(inputs)
    return control_signal[0][0]

# Exemplo de uso:
setpoint = 100
current_speed = 90
control = ml_control(setpoint, current_speed)
print(f"Controle ML: {control}")
