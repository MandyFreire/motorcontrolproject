import serial
import pyqtgraph as pg
from PyQt5 import QtWidgets

arduino = serial.Serial('COM3', 9600)

app = QtWidgets.QApplication([])
win = QtWidgets.QMainWindow()
plot = pg.PlotWidget()
win.setCentralWidget(plot)
win.show()

x, y = [], []

def update():
    rpm = float(arduino.readline().decode().strip())
    x.append(len(x))
    y.append(rpm)
    plot.plot(x, y, clear=True)

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)

app.exec_()

