import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
# from tribo_plot.utils.layout_colorwidget import Color
# from .layout_colorwidget import Color

class AppPlotter(QtWidgets.QMainWindow):
    def __init__(self, buffer_size=5000):
        super().__init__()

        self.setWindowTitle('Tribo Slide Sensor Application')
        self.setFixedSize(QtCore.QSize(1600, 900))

        # Configure pyqtgraph
        pg.setConfigOption('background', 'w')
        
        layout = QtWidgets.QGridLayout()

        # Widget 1: Trajectory (X-Y cumulative sum of velocities)
        self.w1 = pg.PlotWidget()
        self.w1.setLabel('bottom', 'X')
        self.w1.setLabel('left', 'Y')
        self.w1.setTitle('Trajectory (Cumulative Velocity)')
        self.w1.setXRange(-1, 3)
        self.w1.setYRange(-1, 3)
        self.trajectory_plot = self.w1.plot(pen=pg.mkPen('b', width=2), symbol='o', symbolSize=5)

        # Widget 2: Velocity over time (two plots vertically)
        self.w2 = pg.GraphicsLayoutWidget()
        self.p_vx = self.w2.addPlot(row=0, col=0, title='Velocity X (dx)')
        self.p_vy = self.w2.addPlot(row=1, col=0, title='Velocity Y (dy)')
        self.p_vx.setLabel('bottom', 'Time (steps)')
        self.p_vx.setLabel('left', 'dx')
        self.p_vy.setLabel('bottom', 'Time (steps)')
        self.p_vy.setLabel('left', 'dy')
        self.p_vx.setYRange(-1, 1)
        self.p_vy.setYRange(-1, 1)
        self.vel_x_plot = self.p_vx.plot(pen=pg.mkPen('r', width=2))
        self.vel_y_plot = self.p_vy.plot(pen=pg.mkPen('g', width=2))

        # Widget 3: Sensor values (4 channels)
        self.w3 = pg.GraphicsLayoutWidget()
        self.p_s1 = self.w3.addPlot(row=0, col=0, title='Sensor 1')
        self.p_s2 = self.w3.addPlot(row=1, col=0, title='Sensor 2')
        self.p_s3 = self.w3.addPlot(row=2, col=0, title='Sensor 3')
        self.p_s4 = self.w3.addPlot(row=3, col=0, title='Sensor 4')
        
        for p in [self.p_s1, self.p_s2, self.p_s3, self.p_s4]:
            p.setLabel('bottom', 'Time (steps)')
            p.setLabel('left', 'Value')
            p.setYRange(-1500, 1500)
        
        self.sensor_plots = [
            self.p_s1.plot(pen=pg.mkPen('k', width=2)),
            self.p_s2.plot(pen=pg.mkPen((187, 85, 102), width=2)),
            self.p_s3.plot(pen=pg.mkPen((221, 170, 51), width=2)),
            self.p_s4.plot(pen=pg.mkPen((0, 68, 136), width=2)),
        ]

        # Add widgets to layout
        layout.addWidget(self.w1, 0, 0, 9, 9)
        layout.addWidget(self.w2, 0, 9, 9, 4)
        layout.addWidget(self.w3, 0, 13, 9, 3)
        
        container = QtWidgets.QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Data storage
        self.buffer_size = buffer_size
        self.buffer = np.zeros((4, self.buffer_size))

        # initial pose
        self.pose_init = np.zeros((0, 2)) # TEMP

        # Velocity and trajectory data - preallocated circular buffers
        self.max_trajectory_size = 10000  # Preallocate max size
        self.velocity_stack = np.zeros((self.max_trajectory_size, 2), dtype=np.float32)
        self.trajectory = np.zeros((self.max_trajectory_size, 2), dtype=np.float32)
        self.velocity_idx = 0  # Current write index
        self.trajectory_idx = 0
        
        # Sensor data - circular buffers (no Python lists!)
        self.sensor_buffer_size = 5000
        self.sensor_data = np.zeros((4, self.sensor_buffer_size), dtype=np.float32)
        self.sensor_idx = 0  # Circular index for sensor data
        self.sensor_time = np.arange(self.sensor_buffer_size, dtype=np.int32)

        # Timer for updating plots
        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)  # Update every 100ms
        self.timer.timeout.connect(self.update_plots)
        self.timer.start()

    def callback(self, data):
        """Update sensor buffer with new data - Optimized with circular indexing"""
        self.buffer = np.roll(self.buffer, -1, axis=1)
        self.buffer[:, -1] = data
        
        # Update sensor circular buffer (O(1) instead of O(n))
        self.sensor_data[:, self.sensor_idx] = data
        self.sensor_idx = (self.sensor_idx + 1) % self.sensor_buffer_size

    def add_velocity(self, vel):
        """Add velocity to preallocated buffer - Optimized without vstack"""
        vel = np.array(vel, dtype=np.float32)
        
        # Direct assignment to preallocated array
        self.velocity_stack[self.velocity_idx] = vel
        
        # Calculate trajectory point
        if self.velocity_idx == 0:
            trajectory_point = vel
        else:
            trajectory_point = self.trajectory[self.velocity_idx - 1] + vel
        
        self.trajectory[self.trajectory_idx] = trajectory_point
        
        # Increment indices
        self.velocity_idx += 1
        self.trajectory_idx += 1

    def reset(self):
        """Reset trajectory and velocity when detach/idle - Optimized"""
        self.velocity_idx = 0
        self.trajectory_idx = 0
        # Arrays remain preallocated, just reset indices

    def update_plots(self):
        """Update all plots with current data - Optimized"""
        # Get actual slices (avoid copying)
        vel_len = self.velocity_idx
        traj_len = self.trajectory_idx
        
        # Update trajectory plot with only filled portion
        if traj_len > 0:
            traj = self.trajectory[:traj_len]
            self.trajectory_plot.setData(traj[:, 0], traj[:, 1])
            
            # Auto-scale
            # x_min, x_max = traj[:, 0].min(), traj[:, 0].max()
            # y_min, y_max = traj[:, 1].min(), traj[:, 1].max()
            x_min, x_max = -0.03, 0.03
            y_min, y_max = -0.03, 0.03
            
            margin_x = (x_max - x_min) * 0.1 if x_max > x_min else 0.5
            margin_y = (y_max - y_min) * 0.1 if y_max > y_min else 0.5
            self.w1.setXRange(x_min - margin_x, x_max + margin_x)
            self.w1.setYRange(y_min - margin_y, y_max + margin_y)
        
        # Update velocity plots with only filled portion
        if vel_len > 0:
            time_steps = np.arange(vel_len)
            self.vel_x_plot.setData(time_steps, self.velocity_stack[:vel_len, 0])
            self.vel_y_plot.setData(time_steps, self.velocity_stack[:vel_len, 1])
        
        # Update sensor plots - use circular buffer view
        # Reshape data to display in order (handle circular wrap-around)
        sensor_display = np.roll(self.sensor_data, self.sensor_buffer_size - self.sensor_idx, axis=1)
        for i in range(4):
            self.sensor_plots[i].setData(self.sensor_time, sensor_display[i])


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    main = AppPlotter()
    main.show()
    app.exec()