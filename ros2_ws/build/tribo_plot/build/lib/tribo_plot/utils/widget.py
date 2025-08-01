import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
from tribo_plot.utils.layout_colorwidget import Color
# from .layout_colorwidget import Color

class AppPlotter(QtWidgets.QMainWindow):
    def __init__(self, buffer_size=5000):
        super().__init__()

        self.setWindowTitle('Tribo Slide Sensor Application')
        self.setFixedSize(QtCore.QSize(800, 600))

        layout = QtWidgets.QGridLayout()

        # EXAMPLE
        layout.addWidget(Color('red'), 0, 0)
        layout.addWidget(Color('green'), 1, 0)
        layout.addWidget(Color('blue'), 0, 1, 2, 1)
        layout.addWidget(Color('purple'), 2, 0, 1, 2)

        container = QtWidgets.QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)     

        self.buffer_size = buffer_size
        self.buffer = np.zeros((4, self.buffer_size))

        self.pose = np.zeros((2,))
        self.trajectory = np.zeros((0,2))

    def callback(self, data):
        self.buffer = np.roll(self.buffer, -1, axis=1)
        self.buffer[:,-1] = data

    def reset(self):
        self.pose = np.zeros((2,))
        self.trajectory = np.zeros((0,2))

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    main = AppPlotter()
    main.show()
    app.exec()