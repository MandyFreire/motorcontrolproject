import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense

data = np.load("motor_data.npy")
X = data[:, :3]  # setpoint, speed, control_signal
y = data[:, 3]   # error ou próximo controle

model = Sequential([
    Dense(32, activation='relu', input_shape=(3,)),
    Dense(32, activation='relu'),
    Dense(1)  # saída controle
])

model.compile(optimizer='adam', loss='mse')
model.fit(X, y, epochs=50, batch_size=32)
model.save("trained_model.h5")
