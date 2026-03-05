import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class PendulumPublisher(Node):
    def __init__(self):
        super().__init__('pendulum_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/pendulum_target', 10)
        # Publish at 20Hz (0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info('Pendulum Publisher started. Publishing to /pendulum_target')

    def timer_callback(self):
        t = time.time() - self.start_time
        
        # Motion parameters
        center_x = 0.3
        center_y = 0.0
        center_z = 0.2
        amplitude = 0.15 # meters
        frequency = 0.5 # Hz (0.5 cycles per second)
        
        # Calculate target Y based on sine wave
        y = center_y + amplitude * math.sin(2 * math.pi * frequency * t)
        
        # Calculate velocity (derivative of position) to help the mover script
        # v = d/dt (A * sin(wt)) = A * w * cos(wt)
        omega = 2 * math.pi * frequency
        vy = amplitude * omega * math.cos(omega * t)
        speed = abs(vy)
        
        # Ensure a minimum speed so the mover doesn't calculate infinite duration
        if speed < 0.1: speed = 0.1
        
        msg = Float64MultiArray()
        # Format: [x, y, z, velocity]
        msg.data = [center_x, y, center_z, speed]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PendulumPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()