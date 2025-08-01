import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SensorT_Publisher(Node):
    def __init__(self):
        super().__init__('sensorT_fake_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'sensorT_fake', 10)
        timer_period = 0.001 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

        # Sensor Data
        filename = './src/tribo_plot/tribo_plot/data/Log_Sensor_T_.txt'
        try:
            with open(filename, 'r') as f:
                self.lines = [line.strip() for line in f if line.strip()]
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {filename}')
            self.lines = []

    def timer_callback(self):
        if self.count >= len(self.lines):
            self.get_logger().info('End of file reached.')
            self.destrony_node()
            rclpy.shutdown()
            return
        
        line = self.lines[self.count].split(',')
        data = [float(line[i]) for i in range(1, 5)]
        msg = Float32MultiArray(data=data)
        self.publisher_.publish(msg)
        self.get_logger().info(f'{msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node= SensorT_Publisher()
    rclpy.spin(node)
    node.destropy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()